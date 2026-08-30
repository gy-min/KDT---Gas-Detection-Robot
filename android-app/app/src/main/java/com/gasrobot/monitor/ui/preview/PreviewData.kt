package com.gasrobot.monitor.ui.preview

import com.gasrobot.monitor.data.model.ConnectionState
import com.gasrobot.monitor.data.model.DirectiveInfo
import com.gasrobot.monitor.data.model.EmergencyInfo
import com.gasrobot.monitor.data.model.RobotMarker
import com.gasrobot.monitor.data.model.Zone
import com.gasrobot.monitor.data.model.ZoneStatus
import com.gasrobot.monitor.ui.viewmodel.MonitoringUiState

/** Fake data for @Preview only — never used at runtime. Lets you see every screen in
 *  Android Studio's design pane without an emulator or a live server connection. 9 QR points
 *  (A1..C3), mostly normal with one caution/danger point each for the two preview states. */
object PreviewData {

    // zone_id는 QR 지점 단위입니다 (zone_A1/A2/A3 등, SafeScout_API_계약서.md 기준).
    private val zonesSafe = listOf(
        Zone("zone_A1", "Zone A1", intensity = 15, status = ZoneStatus.NORMAL),
        Zone("zone_A2", "Zone A2", intensity = 14, status = ZoneStatus.NORMAL),
        Zone("zone_A3", "Zone A3", intensity = 16, status = ZoneStatus.NORMAL),
        Zone("zone_B1", "Zone B1", intensity = 12, status = ZoneStatus.NORMAL),
        Zone("zone_B2", "Zone B2", intensity = 11, status = ZoneStatus.NORMAL),
        Zone("zone_B3", "Zone B3", intensity = 13, status = ZoneStatus.NORMAL),
        Zone("zone_C1", "Zone C1", intensity = 17, status = ZoneStatus.NORMAL),
        Zone("zone_C2", "Zone C2", intensity = 18, status = ZoneStatus.NORMAL),
        Zone("zone_C3", "Zone C3", intensity = 15, status = ZoneStatus.NORMAL),
    )

    private val zonesEmergency = listOf(
        Zone("zone_A1", "Zone A1", intensity = 15, status = ZoneStatus.NORMAL),
        Zone("zone_A2", "Zone A2", intensity = 14, status = ZoneStatus.NORMAL),
        Zone("zone_A3", "Zone A3", intensity = 16, status = ZoneStatus.NORMAL),
        Zone("zone_B1", "Zone B1", intensity = 12, status = ZoneStatus.NORMAL),
        Zone("zone_B2", "Zone B2", intensity = 52, status = ZoneStatus.CAUTION),
        Zone("zone_B3", "Zone B3", intensity = 13, status = ZoneStatus.NORMAL),
        Zone("zone_C1", "Zone C1", intensity = 17, status = ZoneStatus.NORMAL),
        Zone("zone_C2", "Zone C2", intensity = 18, status = ZoneStatus.NORMAL),
        Zone("zone_C3", "Zone C3", intensity = 3600, status = ZoneStatus.DANGER),
    )

    private val robot = RobotMarker(locationLabel = "(3,5)", statusLabel = "순찰 중")

    val safeUiState = MonitoringUiState(
        zones = zonesSafe,
        robotMarker = robot,
        emergency = EmergencyInfo(active = false),
        connectionState = ConnectionState.CONNECTED
    )

    val emergencyUiState = MonitoringUiState(
        zones = zonesEmergency,
        robotMarker = robot,
        emergency = EmergencyInfo(
            active = true,
            gasType = "메탄 CH₄",
            zoneLabel = "Zone C3",
            ppm = 3600,
            riskSummary = "Zone C3 메탄 농도 급상승 · 대응 진행 중",
            recommendedAction = "누출원 인근 통로 봉쇄 권고. 인원을 개방부로 유도하세요."
        ),
        directive = DirectiveInfo(
            id = "d1",
            message = "3층 전원 대피, 2층은 대기 바랍니다.",
            issuedAtMillis = System.currentTimeMillis()
        ),
        connectionState = ConnectionState.CONNECTED
    )
}
