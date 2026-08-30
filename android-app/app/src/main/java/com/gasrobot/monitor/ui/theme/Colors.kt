package com.gasrobot.monitor.ui.theme

import androidx.compose.ui.graphics.Color

/** Pulled directly from GasRobot.dc.html so the built app matches the approved design. */
object AppColors {
    val PageBg = Color(0xFFEEF1F4)
    val PageBgEmergency = Color(0xFFFDF6F5)
    val Ink = Color(0xFF1F2932)
    val InkFaint = Color(0xFF7B8791)
    val Border = Color(0xFFE5EAEE)

    val Primary = Color(0xFF1466A3)

    val SafeBg = Color(0xFFE9F6EE)
    val SafeBorder = Color(0xFFC1E3CE)
    val SafeIcon = Color(0xFF3FA06A)
    val SafeText = Color(0xFF1C6B43)

    val CautionFill = Color(0xFFFDF3C4)
    val CautionBorder = Color(0xFFD3A500)

    val WarningFill = Color(0xFFFCD9AD)
    val WarningBorder = Color(0xFFE2751F)

    val DangerBg = Color(0xFFFDECEA)
    val DangerBorder = Color(0xFFF4B8B1)
    val DangerIcon = Color(0xFFD62B22)
    val DangerText = Color(0xFF9C1E17)
    val DangerFill = Color(0xFFF7B0AA)

    val ChromeEmergencyBg = Color(0xFF7A0D06)
    val AlarmPulseA = Color(0xFFB31D13)
    val AlarmPulseB = Color(0xFF7A0D06)

    /** Border/text color per zone status — matches the admin web map's 정상(초록)/주의(주황)/
     *  위험(빨강) legend exactly. */
    fun zoneColor(status: com.gasrobot.monitor.data.model.ZoneStatus): Color = when (status) {
        com.gasrobot.monitor.data.model.ZoneStatus.NORMAL -> ZoneMap.Normal
        com.gasrobot.monitor.data.model.ZoneStatus.CAUTION -> ZoneMap.Caution
        com.gasrobot.monitor.data.model.ZoneStatus.DANGER -> ZoneMap.Danger
    }
}

/** Dark palette for the zone map (EvacScreen), pulled to match the admin web's
 *  "평면도 · 로봇 위치" screenshot: near-black background, faint grid, colored zone borders. */
object ZoneMap {
    val Background = Color(0xFF0B0F0D)
    val GridLine = Color(0xFF1C2420)
    val CorridorBar = Color(0xFF6B7280)
    val TextPrimary = Color(0xFFF5F7F6)
    val TextSecondary = Color(0xFF9CA3AF)

    val Normal = Color(0xFF3ECF8E)
    val Caution = Color(0xFFE2A33D)
    val Danger = Color(0xFFE5484D)

    fun fillFor(status: com.gasrobot.monitor.data.model.ZoneStatus): Color = when (status) {
        com.gasrobot.monitor.data.model.ZoneStatus.NORMAL -> Color(0x1A3ECF8E)
        com.gasrobot.monitor.data.model.ZoneStatus.CAUTION -> Color(0x26E2A33D)
        com.gasrobot.monitor.data.model.ZoneStatus.DANGER -> Color(0x33E5484D)
    }
}
