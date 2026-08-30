package com.gasrobot.monitor.ui.viewmodel

import com.gasrobot.monitor.data.model.ConnectionState
import com.gasrobot.monitor.data.model.DirectiveInfo
import com.gasrobot.monitor.data.model.EmergencyInfo
import com.gasrobot.monitor.data.model.RobotMarker
import com.gasrobot.monitor.data.model.Zone

/**
 * Everything a screen needs to render, already shaped for the UI. Views only ever depend on
 * this + MonitoringViewModel — never on MonitoringRepository/MonitoringSocket directly.
 *
 * No robot-fleet fields here on purpose: robot status/battery/telemetry belongs to the separate
 * web admin app. `robotMarker` is just a position pin for the zone map, not fleet telemetry.
 */
data class MonitoringUiState(
    val zones: List<Zone> = emptyList(),
    val robotMarker: RobotMarker? = null,
    val emergency: EmergencyInfo = EmergencyInfo(active = false),
    val directive: DirectiveInfo? = null,
    val connectionState: ConnectionState = ConnectionState.CONNECTING
)
