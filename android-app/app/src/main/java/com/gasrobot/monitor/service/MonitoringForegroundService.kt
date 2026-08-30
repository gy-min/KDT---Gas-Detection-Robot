package com.gasrobot.monitor.service

import android.app.Service
import android.content.Context
import android.content.Intent
import android.os.IBinder
import androidx.core.app.NotificationCompat
import com.gasrobot.monitor.GasRobotApp
import com.gasrobot.monitor.alarm.AlarmController
import com.gasrobot.monitor.alarm.DirectiveNotifier
import com.gasrobot.monitor.data.model.ConnectionState
import com.gasrobot.monitor.data.repository.MonitoringRepository
import com.gasrobot.monitor.ui.MainActivity
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.launch

/**
 * Runs for the entire time the app is installed and has ever been opened once: started from
 * GasRobotApp.onCreate() (first launch) and from BootReceiver (every boot after that), and
 * uses START_STICKY so Android restarts it if the system kills it under memory pressure.
 *
 * Owns the single MonitoringRepository instance (so every screen observes the exact same live
 * data) and is the thing that actually fires AlarmController (full emergency alarm) or
 * DirectiveNotifier (lighter admin-instruction ping) when the repository reports a fresh event.
 */
class MonitoringForegroundService : Service() {

    private val serviceJob = SupervisorJob()
    private val scope = CoroutineScope(Dispatchers.Default + serviceJob)

    private lateinit var repository: MonitoringRepository
    private lateinit var alarmController: AlarmController
    private lateinit var directiveNotifier: DirectiveNotifier

    override fun onCreate() {
        super.onCreate()
        repository = getOrCreateRepository()
        alarmController = getOrCreateAlarmController(applicationContext)
        directiveNotifier = DirectiveNotifier(applicationContext)

        startForeground(ONGOING_NOTIFICATION_ID, buildOngoingNotification())

        repository.start()

        scope.launch {
            repository.emergencyStarted.collect { startedAtMs ->
                if (startedAtMs > 0L) {
                    alarmController.trigger(scope, repository.snapshot.value.emergency)
                }
            }
        }
        scope.launch {
            repository.snapshot.collect { snap ->
                if (!snap.emergency.active) alarmController.stop()
            }
        }
        scope.launch {
            repository.newDirective.collect { directive ->
                if (directive != null) directiveNotifier.notify(directive)
            }
        }
        scope.launch {
            repository.connectionState.collect { updateOngoingNotification(it) }
        }
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int = START_STICKY

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onDestroy() {
        repository.stop()
        scope.cancel()
        super.onDestroy()
    }

    private fun buildOngoingNotification(connection: ConnectionState = ConnectionState.CONNECTING) =
        NotificationCompat.Builder(this, GasRobotApp.CHANNEL_MONITORING)
            .setSmallIcon(android.R.drawable.ic_menu_compass)
            .setContentTitle("정상 감시 중")
            .setContentText(
                when (connection) {
                    ConnectionState.CONNECTED -> "서버와 실시간 연결됨"
                    ConnectionState.CONNECTING -> "서버 연결 중…"
                    ConnectionState.DISCONNECTED -> "연결 끊김 · 재연결 시도 중"
                }
            )
            .setContentIntent(
                android.app.PendingIntent.getActivity(
                    this, 0, Intent(this, MainActivity::class.java),
                    android.app.PendingIntent.FLAG_UPDATE_CURRENT or android.app.PendingIntent.FLAG_IMMUTABLE
                )
            )
            .setOngoing(true)
            .setPriority(NotificationCompat.PRIORITY_LOW)
            .build()

    private fun updateOngoingNotification(connection: ConnectionState) {
        val nm = androidx.core.app.NotificationManagerCompat.from(this)
        runCatching { nm.notify(ONGOING_NOTIFICATION_ID, buildOngoingNotification(connection)) }
    }

    companion object {
        private const val ONGOING_NOTIFICATION_ID = 42

        // No @Volatile/synchronized here: both callers (Service.onCreate, ViewModelProvider.Factory)
        // always run on the main thread, so there's no real race to guard against.
        private var sharedRepository: MonitoringRepository? = null
        private var sharedAlarmController: AlarmController? = null

        /** Shared with the UI layer so Compose screens all observe the same live flows
         *  without going through a bound-service dance. */
        fun getOrCreateRepository(): MonitoringRepository =
            sharedRepository ?: MonitoringRepository(
                CoroutineScope(Dispatchers.Default + SupervisorJob())
            ).also { sharedRepository = it }

        /** One AlarmController for the whole app. Important: the siren is a live AudioTrack
         *  owned by whichever instance called trigger() — if the ViewModel used a *different*
         *  instance than the service, calling stop() from the UI wouldn't actually silence the
         *  audio the service started. Sharing one instance avoids that. */
        fun getOrCreateAlarmController(context: Context): AlarmController =
            sharedAlarmController ?: AlarmController(context.applicationContext)
                .also { sharedAlarmController = it }

        fun start(context: Context) {
            val intent = Intent(context, MonitoringForegroundService::class.java)
            context.startForegroundService(intent)
        }
    }
}
