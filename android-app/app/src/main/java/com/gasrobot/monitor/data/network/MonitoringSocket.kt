package com.gasrobot.monitor.data.network

import android.util.Log
import com.gasrobot.monitor.BuildConfig
import com.gasrobot.monitor.data.model.ConnectionState
import com.gasrobot.monitor.data.model.RealtimeEnvelopeDto
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.serialization.json.Json
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import okhttp3.Response as OkResponse
import java.util.concurrent.TimeUnit

private const val TAG = "MonitoringSocket"

/**
 * Connects to the real `/ws/realtime` endpoint (main.py's `websocket_realtime`). Every message
 * is one event, not a full state snapshot: `{"topic": "sensor/reading"|"robot/event"|"fire/event",
 * "data": {...}}`, pushed exactly once per MQTT message the server relayed. MonitoringRepository
 * is responsible for folding this stream of deltas into the app's running picture of the
 * building — this class's only job is "get envelopes off the wire reliably".
 *
 * Reconnects automatically with capped exponential backoff so the phone keeps trying to
 * re-establish the link for as long as the app/service is alive, without hammering the server.
 *
 * Logs to Logcat (tag "MonitoringSocket") at every stage — connect/open/message/parse-failure/
 * close — since a mismatch between the real server's JSON and what this expects would otherwise
 * fail *silently*. Filter Logcat by that tag while testing against the real backend.
 */
class MonitoringSocket(
    private val scope: CoroutineScope,
    private val json: Json = Json { ignoreUnknownKeys = true }
) {
    private val client = OkHttpClient.Builder()
        .pingInterval(20, TimeUnit.SECONDS) // keeps NAT/load-balancer connections alive
        .build()

    private var socket: WebSocket? = null
    private var reconnectAttempt = 0
    private var stopped = false

    private val _events = MutableSharedFlow<RealtimeEnvelopeDto>(extraBufferCapacity = 16)
    val events = _events.asSharedFlow()

    private val _connectionState = MutableStateFlow(ConnectionState.DISCONNECTED)
    val connectionState = _connectionState.asStateFlow()

    fun connect() {
        stopped = false
        openSocket()
    }

    fun disconnect() {
        stopped = true
        socket?.close(1000, "client shutdown")
        socket = null
        _connectionState.value = ConnectionState.DISCONNECTED
    }

    private fun openSocket() {
        _connectionState.value = ConnectionState.CONNECTING
        Log.i(TAG, "connecting to ${BuildConfig.WS_URL}")
        val request = Request.Builder().url(BuildConfig.WS_URL).build()
        socket = client.newWebSocket(request, object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: OkResponse) {
                Log.i(TAG, "connected (HTTP ${response.code})")
                reconnectAttempt = 0
                _connectionState.value = ConnectionState.CONNECTED
            }

            override fun onMessage(webSocket: WebSocket, text: String) {
                Log.d(TAG, "message received: $text")
                runCatching { json.decodeFromString<RealtimeEnvelopeDto>(text) }
                    .onSuccess { scope.launch { _events.emit(it) } }
                    .onFailure { e ->
                        Log.e(TAG, "failed to parse realtime message — expected {\"topic\":..,\"data\":{..}}", e)
                    }
            }

            override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
                Log.i(TAG, "closing: $code $reason")
                webSocket.close(1000, null)
            }

            override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                Log.i(TAG, "closed: $code $reason")
                _connectionState.value = ConnectionState.DISCONNECTED
                scheduleReconnect()
            }

            override fun onFailure(webSocket: WebSocket, t: Throwable, response: OkResponse?) {
                Log.e(TAG, "connection failure (HTTP ${response?.code})", t)
                _connectionState.value = ConnectionState.DISCONNECTED
                scheduleReconnect()
            }
        })
    }

    private fun scheduleReconnect() {
        if (stopped) return
        reconnectAttempt++
        val backoffMs = minOf(30_000L, 1_000L * (1 shl minOf(reconnectAttempt, 5)))
        Log.i(TAG, "reconnecting in ${backoffMs}ms (attempt $reconnectAttempt)")
        scope.launch {
            delay(backoffMs)
            if (!stopped) openSocket()
        }
    }
}
