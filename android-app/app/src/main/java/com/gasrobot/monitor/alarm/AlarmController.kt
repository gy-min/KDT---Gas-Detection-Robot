package com.gasrobot.monitor.alarm

import android.app.Notification
import android.app.PendingIntent
import android.content.Context
import android.content.Intent
import android.os.Build
import android.os.VibrationEffect
import android.os.Vibrator
import androidx.core.app.NotificationCompat
import androidx.core.app.NotificationManagerCompat
import androidx.core.content.ContextCompat
import com.gasrobot.monitor.GasRobotApp
import com.gasrobot.monitor.data.model.EmergencyInfo
import com.gasrobot.monitor.ui.EmergencyActivity
import kotlinx.coroutines.CoroutineScope

/**
 * Makes an emergency genuinely hard to miss, on purpose:
 *  1) Continuous vibration pattern (repeats until dismissed).
 *  2) Synthesized siren, looped (SirenPlayer), at 알림/벨소리(notification/ringtone) stream
 *     volume rather than media volume — people routinely carry media volume low or muted, but
 *     leave ringtone/notification volume audible, so this is far more likely to actually be
 *     heard.
 *  3) A high-priority notification with a fullScreenIntent, so on a locked or backgrounded
 *     phone Android itself launches EmergencyActivity full-screen — the user doesn't have to
 *     go find and open a notification to see it.
 *  4) EmergencyActivity blocks with an explicit "상황 확인하기" acknowledgement action; nothing
 *     stops the siren/vibration until that's tapped (see EmergencyActivity + stop()).
 *
 * This is intentionally aggressive because the whole point (per the product requirement) is
 * that "I didn't notice" shouldn't be a plausible excuse for a gas-leak alert.
 */
class AlarmController(private val context: Context) {

    private val siren = SirenPlayer()
    private val vibrator: Vibrator by lazy { context.systemVibrator() }

    fun trigger(scope: CoroutineScope, info: EmergencyInfo) {
        siren.start(scope)
        startVibration()
        postFullScreenNotification(info)
    }

    fun stop() {
        siren.stop()
        vibrator.cancel()
        NotificationManagerCompat.from(context).cancel(EMERGENCY_NOTIFICATION_ID)
    }

    private fun startVibration() {
        val pattern = GasRobotApp.EMERGENCY_VIBRATION_PATTERN
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            vibrator.vibrate(VibrationEffect.createWaveform(pattern, 0)) // repeat=0 -> loops from index 0
        } else {
            @Suppress("DEPRECATION")
            vibrator.vibrate(pattern, 0)
        }
    }

    private fun postFullScreenNotification(info: EmergencyInfo) {
        val fullScreenIntent = Intent(context, EmergencyActivity::class.java).apply {
            flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TOP
        }
        val fullScreenPendingIntent = PendingIntent.getActivity(
            context, 0, fullScreenIntent,
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        )

        val notification = NotificationCompat.Builder(context, GasRobotApp.CHANNEL_EMERGENCY)
            .setSmallIcon(android.R.drawable.stat_sys_warning)
            .setContentTitle("비상 · 가스 누출 감지")
            .setContentText("${info.zoneLabel} · ${info.gasType} · ${info.ppm} ppm")
            .setPriority(NotificationCompat.PRIORITY_MAX)
            .setCategory(NotificationCompat.CATEGORY_ALARM)
            .setFullScreenIntent(fullScreenPendingIntent, true)
            .setContentIntent(fullScreenPendingIntent)
            .setAutoCancel(false)
            .setOngoing(true)
            .setVisibility(NotificationCompat.VISIBILITY_PUBLIC)
            .build()

        // Also launch directly — fullScreenIntent is the reliable path when the screen is off
        // or the app isn't foregrounded, but launching now too covers the "phone is already
        // unlocked and in front of them" case without waiting on system notification timing.
        context.startActivity(fullScreenIntent)

        ContextCompat.getSystemService(context, android.app.NotificationManager::class.java)
            ?.notify(EMERGENCY_NOTIFICATION_ID, notification)
    }

    companion object {
        const val EMERGENCY_NOTIFICATION_ID = 1001
    }
}
