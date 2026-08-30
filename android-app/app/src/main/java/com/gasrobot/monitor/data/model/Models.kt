package com.gasrobot.monitor.data.model

import kotlinx.serialization.Serializable

/** Zone status tier, mirrors the admin web map's legend exactly (정상/주의/위험 — 3 tiers,
 *  no separate "경계" tier). The server decides the tier (it already knows the sensor
 *  calibration); the app just renders whatever status string it's given. */
enum class ZoneStatus { NORMAL, CAUTION, DANGER }

/** One room/zone on the floor plan, matching the admin web's "평면도 · 로봇 위치" map:
 *  a named rectangle (Zone A, Zone B, ...) with a gas-intensity reading and a status that
 *  drives its border/background color. `note` is for the odd extra line some zones show
 *  (e.g. "불꽃 미감지" on Zone D) — the real `fixed_sensor_reading` table has no such column,
 *  so this stays null until/unless the server starts sending one (e.g. by joining in the
 *  latest fire_event per zone). */
@Serializable
data class Zone(
    val id: String,
    val name: String,
    val intensity: Int,
    val status: ZoneStatus,
    val note: String? = null,
    /** Raw ISO timestamp string from fixed_sensor_reading.reading_time, as returned by the
     *  server (Python's datetime.isoformat()). Null until a real reading has come in. */
    val updatedAt: String? = null
)

/** A robot's last known position — just enough to show "a robot is here, doing X", not the
 *  full fleet-management telemetry (battery/signal/etc), which lives in the separate web admin
 *  app. Real robots report a free-form "(x,y)" QR coordinate (`robot_event.location`), not a
 *  fixed zone id, so this is rendered as a caption under the map rather than pinned inside a
 *  specific zone card. */
@Serializable
data class RobotMarker(
    val locationLabel: String,
    val statusLabel: String
)

/** Which sensor system triggered the emergency — same siren/vibration/full-screen alarm
 *  treatment either way (per product decision), but the wording on screen needs to say
 *  "화재" not "가스 누출" when it's actually a fire_event, so this rides along. */
enum class EmergencyKind { GAS, FIRE }

@Serializable
data class EmergencyInfo(
    val active: Boolean,
    val kind: EmergencyKind = EmergencyKind.GAS,
    val gasType: String = "메탄 CH₄",
    val zoneLabel: String = "",
    val ppm: Int = 0,
    val riskSummary: String = "",
    val recommendedAction: String = ""
)

/** A free-text instruction the admin (web) side broadcasts to every employee's phone —
 *  e.g. "3층 전원 대피, 2층은 대기". Separate from EmergencyInfo because it's a human typing
 *  something in real time, not an automatic sensor-driven alarm; it gets a normal high-priority
 *  notification + short vibration, not the full siren/full-screen treatment. `id` changing is
 *  what tells the app "this is a new instruction, notify again" — see MonitoringRepository.
 *
 *  Backed by the real server's `staff_instruction` table: `POST /api/instructions` (admin web)
 *  saves a row and broadcasts it over `/ws/realtime` as `{"topic": "instruction/new", ...}`;
 *  `GET /api/instructions` is the REST bootstrap/fallback. MonitoringRepository maps
 *  `InstructionDto` -> `DirectiveInfo`. */
@Serializable
data class DirectiveInfo(
    val id: String,
    val message: String,
    val issuedAtMillis: Long = 0L
)

/** Full real-time snapshot pushed by the backend over the WebSocket (or fetched via REST
 *  as a fallback / initial paint before the socket connects). Robot-fleet management data
 *  (battery, signal, per-robot status) lives in the separate web admin app and isn't part of
 *  this employee-facing snapshot — the phone only needs the zone map, emergency state, and
 *  any admin directive. */
@Serializable
data class MonitoringSnapshot(
    val zones: List<Zone> = emptyList(),
    val robotMarker: RobotMarker? = null,
    val emergency: EmergencyInfo = EmergencyInfo(active = false),
    val directive: DirectiveInfo? = null,
    val serverTimeMillis: Long = 0L
)

/** Connection health, surfaced in the UI so an operator can tell "no data" apart from "safe". */
enum class ConnectionState { CONNECTING, CONNECTED, DISCONNECTED }

/** Matches main.py's AuthRequest pydantic model exactly — same body shape for register and
 *  login. `role` is hardcoded to "staff" from this app; employees never see a role picker
 *  (that's only meaningful for the separate web admin's accounts). */
@Serializable
data class AuthRequest(
    val username: String,
    val password: String,
    val role: String = "staff"
)

/** What both /api/auth/register and /api/auth/login return on success — the whole user row,
 *  no token (the server doesn't issue one). This is genuinely all the "session" there is;
 *  SessionPrefs just remembers it locally so the app doesn't ask again every launch. */
@Serializable
data class AuthUser(
    val id: Int,
    val username: String,
    val role: String
)
