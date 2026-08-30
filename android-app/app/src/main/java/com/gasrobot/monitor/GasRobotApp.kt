package com.gasrobot.monitor

import android.app.Application
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.content.Context
import android.os.Build
import androidx.core.content.ContextCompat
import com.gasrobot.monitor.service.MonitoringForegroundService

/**
 * Entry point. Two responsibilities on launch:
 *  1) Register the two notification channels (routine monitoring vs. emergency alarm).
 *     The emergency channel is set up so it CANNOT be muted/downgraded by the user in a way
 *     that silently drops the alert (IMPORTANCE_HIGH, bypasses DND-safe defaults, has its own
 *     alarm-like sound so it doesn't inherit whatever the phone's default notification tone is).
 *  2) Kick off the persistent MonitoringForegroundService so the app keeps talking to the
 *     backend (and can therefore keep evaluating risk / firing alarms) even while backgrounded.
 *
 * NOTE: Android cannot start a foreground service the instant the app is *installed* (no launch
 * has happened yet, so there's no process). What we can do — and do here — is: (a) start the
 * service the first time the app is opened, and (b) restart it automatically after every device
 * boot via BootReceiver, so that from the first launch onward the service effectively never
 * stops running.
 */
class GasRobotApp : Application() {

    override fun onCreate() {
        super.onCreate()
        createNotificationChannels()
        MonitoringForegroundService.start(this)
    }

    private fun createNotificationChannels() {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O) return
        val nm = ContextCompat.getSystemService(this, NotificationManager::class.java) ?: return

        val monitoring = NotificationChannel(
            CHANNEL_MONITORING,
            getString(R.string.monitoring_channel_name),
            NotificationManager.IMPORTANCE_LOW
        ).apply {
            description = "정상 감시 중 상태를 보여주는 지속 알림"
            setShowBadge(false)
        }

        val emergency = NotificationChannel(
            CHANNEL_EMERGENCY,
            getString(R.string.emergency_channel_name),
            NotificationManager.IMPORTANCE_HIGH
        ).apply {
            description = "가스 누출 감지 시 즉시 확인이 필요한 경보"
            enableVibration(true)
            vibrationPattern = EMERGENCY_VIBRATION_PATTERN
            enableLights(true)
            lockscreenVisibility = Notification.VISIBILITY_PUBLIC
            // Sound is attached per-notification (see AlarmController) using our synthesized
            // siren so it doesn't depend on the user's default ringtone.
            setBypassDnd(true)
        }

        val directive = NotificationChannel(
            CHANNEL_DIRECTIVE,
            getString(R.string.directive_channel_name),
            NotificationManager.IMPORTANCE_HIGH
        ).apply {
            description = "관리자가 직접 보내는 실시간 지시사항"
            enableVibration(true)
            vibrationPattern = DIRECTIVE_VIBRATION_PATTERN
            lockscreenVisibility = Notification.VISIBILITY_PUBLIC
        }

        nm.createNotificationChannel(monitoring)
        nm.createNotificationChannel(emergency)
        nm.createNotificationChannel(directive)
    }

    companion object {
        const val CHANNEL_MONITORING = "monitoring_status"
        const val CHANNEL_EMERGENCY = "gas_leak_emergency"
        const val CHANNEL_DIRECTIVE = "admin_directive"
        val EMERGENCY_VIBRATION_PATTERN = longArrayOf(0, 300, 140, 300, 140, 600, 140, 300)
        val DIRECTIVE_VIBRATION_PATTERN = longArrayOf(0, 200, 100, 200)
    }
}
