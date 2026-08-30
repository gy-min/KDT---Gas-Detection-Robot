package com.gasrobot.monitor.data.repository

import com.gasrobot.monitor.data.model.ConnectionState
import com.gasrobot.monitor.data.model.DirectiveInfo
import com.gasrobot.monitor.data.model.EmergencyInfo
import com.gasrobot.monitor.data.model.EmergencyKind
import com.gasrobot.monitor.data.model.FireEventDto
import com.gasrobot.monitor.data.model.InstructionDto
import com.gasrobot.monitor.data.model.MonitoringSnapshot
import com.gasrobot.monitor.data.model.RealtimeTopics
import com.gasrobot.monitor.data.model.RobotEventDto
import com.gasrobot.monitor.data.model.RobotMarker
import com.gasrobot.monitor.data.model.Zone
import com.gasrobot.monitor.data.model.ZoneReadingDto
import com.gasrobot.monitor.data.model.ZoneStatus
import com.gasrobot.monitor.data.model.formatZoneName
import com.gasrobot.monitor.data.model.isDangerLevel
import com.gasrobot.monitor.data.model.statusFromKorean
import com.gasrobot.monitor.data.network.ApiService
import com.gasrobot.monitor.data.network.MonitoringSocket
import com.gasrobot.monitor.data.network.NetworkModule
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.decodeFromJsonElement

/**
 * Single source of truth for the whole app. Runs for as long as MonitoringForegroundService
 * is alive (i.e. continuously, from first app launch onward).
 *
 * Unlike the earlier fictional "/monitoring/snapshot" design, the real server
 * (main.py/database.py) doesn't push a full state snapshot — REST gives point-in-time reads of
 * individual tables (`fixed_sensor_reading`, `robot_event`, `fire_event`), and the WebSocket
 * pushes one event at a time (`{"topic": ..., "data": {...}}`), one per MQTT message relayed.
 * So this class keeps its own small in-memory model — the latest reading per zone, the last
 * robot event, the last fire event — and rebuilds MonitoringSnapshot from it every time any
 * piece changes:
 *  - Cold start / whenever the WebSocket is disconnected: REST-poll all three endpoints.
 *  - While connected: fold each WebSocket envelope into the matching piece of state as it
 *    arrives (sensor/reading -> upsert that zone; robot/event -> update the robot marker;
 *    fire/event -> update fire status), which is cheap and doesn't require a full re-fetch.
 *
 * `emergency.active` isn't a field the server sends — there's no such column anywhere. It's
 * derived here: true if any zone's status is 위험(DANGER), or the latest robot/fire event's
 * alert_level reads as dangerous. See [isDangerLevel]/[statusFromKorean].
 *
 * Also detects two kinds of "new event, not just a value" edges and republishes them as
 * one-shot signals so the alarm layer reacts exactly once per event, not once per update:
 *  - `emergency.active` false->true  => fire the full siren/vibration/full-screen alarm.
 *  - `directive.id` changing         => a new admin instruction arrived, either from the
 *    `instruction/new` WebSocket push or the `/api/instructions` REST bootstrap.
 */
class MonitoringRepository(
    private val scope: CoroutineScope,
    private val api: ApiService = NetworkModule.apiService
) {
    private val socket = MonitoringSocket(scope)
    private val json = Json { ignoreUnknownKeys = true }

    private val _snapshot = MutableStateFlow(MonitoringSnapshot())
    val snapshot = _snapshot.asStateFlow()

    val connectionState = socket.connectionState

    private val _emergencyStarted = MutableStateFlow(0L) // increments -> "fire the alarm now"
    val emergencyStarted = _emergencyStarted.asStateFlow()

    private val _newDirective = MutableStateFlow<DirectiveInfo?>(null) // changes -> "notify now"
    val newDirective = _newDirective.asStateFlow()

    // ---- in-memory model built from REST bootstrap + WebSocket deltas ----
    private val zoneReadings = linkedMapOf<String, ZoneReadingDto>() // keyed by zone_id; sorted by id when published
    private var lastRobotEvent: RobotEventDto? = null
    private var lastFireEvent: FireEventDto? = null
    private var lastInstruction: InstructionDto? = null

    private var wasEmergency = false
    private var lastDirectiveId: String? = null
    private var pollJob: kotlinx.coroutines.Job? = null

    fun start() {
        // The WebSocket only pushes *new* events from here on — it has no "give me current
        // state" message. So an initial REST read is required even if the socket connects
        // instantly, or the zone map would sit empty until the first live sensor reading.
        scope.launch { bootstrapFromRest() }

        socket.connect()
        scope.launch {
            socket.events.collect { envelope ->
                runCatching {
                    when (envelope.topic) {
                        RealtimeTopics.SENSOR_READING -> {
                            val reading = json.decodeFromJsonElement(ZoneReadingDto.serializer(), envelope.data)
                            zoneReadings[reading.zone_id] = reading
                        }
                        RealtimeTopics.ROBOT_EVENT -> {
                            lastRobotEvent = json.decodeFromJsonElement(RobotEventDto.serializer(), envelope.data)
                        }
                        RealtimeTopics.FIRE_EVENT -> {
                            lastFireEvent = json.decodeFromJsonElement(FireEventDto.serializer(), envelope.data)
                        }
                        RealtimeTopics.INSTRUCTION_NEW -> {
                            lastInstruction = json.decodeFromJsonElement(InstructionDto.serializer(), envelope.data)
                        }
                    }
                }
                rebuildAndPublish()
            }
        }
        scope.launch {
            socket.connectionState.collect { state ->
                if (state == ConnectionState.DISCONNECTED) startFallbackPolling()
                else stopFallbackPolling()
            }
        }
    }

    fun stop() {
        socket.disconnect()
        stopFallbackPolling()
    }

    /** One-shot request/response, unlike the rest of this class — the server computes the route
     *  fresh on every call (Dijkstra over the current danger-zone set), there's nothing to fold
     *  into the running snapshot or listen for over the WebSocket. `currentLocation` must be one
     *  of the 12 node ids from the real floor plan (see EvacGraph.kt/map_data.py).
     *  Server failure modes, both surfaced as `Result.failure` here:
     *   - 400: `currentLocation` isn't a valid node id / QR coordinate.
     *   - 404: no reachable exit — e.g. every edge out of the start node is blocked.
     *  Returns the route (node ids, start to exit inclusive) alongside the zone ids the server
     *  routed around for this run. */
    suspend fun fetchEvacuationRoute(currentLocation: String): Result<Pair<List<String>, List<String>>> =
        runCatching {
            val dto = api.getEvacuationRoute(currentLocation)
            dto.route.orEmpty() to dto.blocked_zones.orEmpty()
        }

    private fun startFallbackPolling() {
        if (pollJob?.isActive == true) return
        pollJob = scope.launch {
            while (true) {
                bootstrapFromRest()
                delay(ApiService.POLL_INTERVAL_MS)
            }
        }
    }

    private fun stopFallbackPolling() {
        pollJob?.cancel()
        pollJob = null
    }

    private suspend fun bootstrapFromRest() {
        runCatching { api.getFixedSensorLatest() }.onSuccess { readings ->
            readings.forEach { zoneReadings[it.zone_id] = it }
        }
        runCatching { api.getRobotEvents(limit = 1) }.onSuccess { events ->
            events.firstOrNull()?.let { lastRobotEvent = it }
        }
        runCatching { api.getFireEvents(limit = 1) }.onSuccess { events ->
            events.firstOrNull()?.let { lastFireEvent = it }
        }
        runCatching { api.getInstructions(limit = 1) }.onSuccess { instructions ->
            instructions.firstOrNull()?.let { lastInstruction = it }
        }
        rebuildAndPublish()
    }

    /** Turns the current in-memory model (zoneReadings + lastRobotEvent + lastFireEvent) into
     *  a MonitoringSnapshot and publishes it — called after every REST bootstrap and every
     *  WebSocket delta, so the UI is always looking at a consistent recomputed picture rather
     *  than a partially-applied one. */
    private fun rebuildAndPublish() {
        val zones = zoneReadings.values.sortedBy { it.zone_id }.map { reading ->
            Zone(
                id = reading.zone_id,
                name = formatZoneName(reading.zone_id),
                intensity = (reading.strength ?: reading.rs_value ?: 0.0).toInt(),
                status = statusFromKorean(reading.status),
                updatedAt = reading.reading_time
            )
        }

        val dangerZone = zones.firstOrNull { it.status == ZoneStatus.DANGER }
        val robotAlertDanger = isDangerLevel(lastRobotEvent?.alert_level)
        val fireAlertDanger = isDangerLevel(lastFireEvent?.alert_level) || lastFireEvent?.flame_detected == true

        val active = dangerZone != null || robotAlertDanger || fireAlertDanger
        val emergency = if (!active) {
            EmergencyInfo(active = false)
        } else {
            val kind = if (fireAlertDanger) EmergencyKind.FIRE else EmergencyKind.GAS
            EmergencyInfo(
                active = true,
                kind = kind,
                gasType = if (kind == EmergencyKind.FIRE) "화재" else (lastRobotEvent?.gas_type ?: "가스 누출"),
                zoneLabel = dangerZone?.name ?: lastFireEvent?.location ?: lastRobotEvent?.location ?: "",
                ppm = dangerZone?.intensity ?: 0,
                riskSummary = buildString {
                    if (dangerZone != null) append("${dangerZone.name} 위험 감지")
                    if (fireAlertDanger) append(if (isNotEmpty()) " · 화재 신호 감지" else "화재 신호 감지")
                },
                recommendedAction = "누출/발화 구역 인근 통로 봉쇄 권고. 인원을 개방부로 유도하세요."
            )
        }

        val robotMarker = lastRobotEvent?.location?.let { loc ->
            RobotMarker(
                locationLabel = loc,
                statusLabel = when (lastRobotEvent?.event_type) {
                    "gas_response" -> "출동 중"
                    "patrol" -> "순찰 중"
                    else -> lastRobotEvent?.event_type ?: "대기중"
                }
            )
        }

        val directive = lastInstruction?.let { ins ->
            DirectiveInfo(
                // Prefer the DB row id (stable across REST re-fetches); the live WebSocket push
                // always carries one too since the server assigns it before broadcasting.
                id = ins.id?.toString() ?: ins.instruction.orEmpty(),
                message = if (ins.event_label.isNullOrBlank()) {
                    ins.instruction.orEmpty()
                } else {
                    "[${ins.event_label}] ${ins.instruction.orEmpty()}"
                }
            )
        }

        applySnapshot(
            MonitoringSnapshot(
                zones = zones,
                robotMarker = robotMarker,
                emergency = emergency,
                directive = directive,
                serverTimeMillis = System.currentTimeMillis()
            )
        )
    }

    private fun applySnapshot(snap: MonitoringSnapshot) {
        _snapshot.value = snap

        val nowEmergency = snap.emergency.active
        if (nowEmergency && !wasEmergency) {
            _emergencyStarted.value = System.currentTimeMillis()
        }
        wasEmergency = nowEmergency

        val directive = snap.directive
        if (directive != null && directive.id != lastDirectiveId) {
            lastDirectiveId = directive.id
            _newDirective.value = directive
        }
    }
}
