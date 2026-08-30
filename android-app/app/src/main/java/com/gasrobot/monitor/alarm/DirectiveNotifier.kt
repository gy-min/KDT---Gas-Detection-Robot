package com.gasrobot.monitor.alarm

import android.content.Context
import android.content.Intent
import android.os.Build
import android.os.VibrationEffect
import android.os.Vibrator
import androidx.core.app.NotificationCompat
import androidx.core.content.ContextCompat
import com.gasrobot.monitor.GasRobotApp
import com.gasrobot.monitor.data.model.DirectiveInfo
import com.gasrobot.monitor.ui.MainActivity

/**
 * A new admin directive is a human typing an instruction in real time — it deserves attention,
 * but not the full siren/full-screen/can't-dismiss treatment reserved for an actual gas-leak
 * emergency (AlarmController). This is intentionally lighter: one short buzz + a normal
 * high-priority notification that opens straight to the Home screen (where the latest
 * directive is shown as a banner).
 */
class DirectiveNotifier(private val context: Context) {

    private val vibrator: Vibrator by lazy { context.systemVibrator() }

    fun notify(directive: DirectiveInfo) {
        val pattern = GasRobotApp.DIRECTIVE_VIBRATION_PATTERN
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            vibrator.vibrate(VibrationEffect.createWaveform(pattern, -1)) // -1 = play once, don't loop
        } else {
            @Suppress("DEPRECATION")
            vibrator.vibrate(pattern, -1)
        }

        val openIntent = Intent(context, MainActivity::class.java).apply {
            flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TOP
        }
        val pendingIntent = android.app.PendingIntent.getActivity(
            context, 0, openIntent,
            android.app.PendingIntent.FLAG_UPDATE_CURRENT or android.app.PendingIntent.FLAG_IMMUTABLE
        )

        val notification = NotificationCompat.Builder(context, GasRobotApp.CHANNEL_DIRECTIVE)
            .setSmallIcon(android.R.drawable.ic_dialog_info)
            .setContentTitle("관리자 지시사항")
            .setContentText(directive.message)
            .setStyle(NotificationCompat.BigTextStyle().bigText(directive.message))
            .setPriority(NotificationCompat.PRIORITY_HIGH)
            .setCategory(NotificationCompat.CATEGORY_MESSAGE)
            .setContentIntent(pendingIntent)
            .setAutoCancel(true)
            .build()

        ContextCompat.getSystemService(context, android.app.NotificationManager::class.java)
            ?.notify(DIRECTIVE_NOTIFICATION_ID, notification)
    }

    companion object {
        const val DIRECTIVE_NOTIFICATION_ID = 1002
    }
}
