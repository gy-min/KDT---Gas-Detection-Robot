package com.gasrobot.monitor.service

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent

/** Restarts monitoring after reboot so a device that's been set up once keeps watching
 *  without anyone having to manually relaunch the app. */
class BootReceiver : BroadcastReceiver() {
    override fun onReceive(context: Context, intent: Intent) {
        if (intent.action == Intent.ACTION_BOOT_COMPLETED) {
            MonitoringForegroundService.start(context)
        }
    }
}
