package com.gasrobot.monitor.alarm

import android.content.Context
import android.os.Build
import android.os.Vibrator
import android.os.VibratorManager

/** Shared by AlarmController and DirectiveNotifier — the VIBRATOR_MANAGER_SERVICE API only
 *  exists from API 31 (S) onward, so this is the one place that branches on it. */
fun Context.systemVibrator(): Vibrator =
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
        (getSystemService(Context.VIBRATOR_MANAGER_SERVICE) as VibratorManager).defaultVibrator
    } else {
        @Suppress("DEPRECATION")
        getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
    }
