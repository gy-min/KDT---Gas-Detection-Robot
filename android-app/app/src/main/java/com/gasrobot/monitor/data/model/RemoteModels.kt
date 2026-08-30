package com.gasrobot.monitor.data.model

import kotlinx.serialization.Serializable

/**
 * These mirror the *real* FastAPI server's actual response/message shapes 1:1 (see the
 * uploaded main.py / database.py / models.py) — not the app's internal UI models above.
 * MonitoringRepository is the only place that reads these; it translates them into
 * Zone/RobotMarker/EmergencyInfo before anything in ui/ ever sees them, so a future schema
 * change only touches this file + the repository, not the screens.
 *
 * All fields are optional because:
 *  - `SELECT *` column sets aren't guaranteed here without the actual CREATE TABLE statements
 *  - MQTT payloads go through `model_dump(exclude_none=True)` server-side, so any field can be
 *    legitimately absent
 * kotlinx.serialization requires every field to either have a default or be nullable, so
 * "optional with a safe default" is the only way to not crash on a slightly different row shape.
 */

/** One row from `GET /api/fixed-sensor/latest` (table: fixed_sensor_reading). `status` is a
 *  Korean string ("정상"/"주의"/"위험") straight from the DB — see [statusFromKorean]. */
@Serializable
data class ZoneReadingDto(
    val zone_id: String,
    val rs_value: Double? = null,
    val strength: Double? = null,
    val status: String? = null,
    val reading_time: String? = null
)

/** One row from `GET /api/robot-events` (table: robot_event). `event_type` is "patrol" for
 *  routine movement or "gas_response" when the robot was dispatched to investigate. Gas
 *  classification fields (gas_type/confidence/alert_level) are only present on the *detail*
 *  endpoint (`/api/robot-events/{id}`) or on a `robot/event` WebSocket push where the robot
 *  already finished classifying — the list endpoint's rows won't have them. */
@Serializable
data class RobotEventDto(
    val location: String? = null,
    val event_type: String? = null,
    val gas_type: String? = null,
    val confidence: Double? = null,
    val strength: Double? = null,
    val alert_level: String? = null,
    val event_time: String? = null
)

/** One row from `GET /api/fire-events` (table: fire_event). */
@Serializable
data class FireEventDto(
    val location: String? = null,
    val source: String? = null,
    val flame_detected: Boolean? = null,
    val smoke_detected: Boolean? = null,
    val confidence: Double? = null,
    val alert_level: String? = null,
    val event_time: String? = null
)

/** The envelope every `/ws/realtime` message arrives in: `{"topic": "...", "data": {...}}`.
 *  `data`'s actual shape depends on `topic` (ZoneReadingDto / RobotEventDto / FireEventDto /
 *  InstructionDto), so it's decoded a second time in MonitoringRepository once the topic is
 *  known — see RealtimeTopics below. */
@Serializable
data class RealtimeEnvelopeDto(
    val topic: String,
    val data: kotlinx.serialization.json.JsonElement
)

/** Exact topic strings the server forwards. `sensor/reading`/`robot/event`/`fire/event` come from
 *  config.py's MQTT_TOPICS; `instruction/new` is broadcast directly by `POST /api/instructions`
 *  (not MQTT-sourced), fired right after the row is saved to `staff_instruction`. */
object RealtimeTopics {
    const val SENSOR_READING = "sensor/reading"
    const val ROBOT_EVENT = "robot/event"
    const val FIRE_EVENT = "fire/event"
    const val INSTRUCTION_NEW = "instruction/new"
}

/** Response body of `GET /api/evacuation-route?current_location=<node id>`. `route` is the
 *  ordered list of node ids (e.g. `["C","A","TL"]` — see `EvacGraph.kt`/`map_data.py` for the
 *  node graph) from `current_location` to the nearest exit (inclusive of both ends), already
 *  computed server-side by Dijkstra. `blocked_zones` lists which `zone_id`s were excluded from
 *  the graph for this run (per SafeScout_API_계약서.md). A 400 (bad location) or 404 (no
 *  reachable exit) never reaches this class; Retrofit throws an `HttpException` instead, which
 *  `MonitoringRepository.fetchEvacuationRoute` converts into a `Result.failure`. */
@Serializable
data class EvacuationRouteDto(
    val start: String? = null,
    val route: List<String>? = null,
    val blocked_zones: List<String>? = null
)

/** One row from `GET /api/instructions` (table: staff_instruction), also the shape of the
 *  `instruction/new` WebSocket push (minus `created_at`, which the live push doesn't include —
 *  MonitoringRepository stamps its own arrival time for that case). `event_kind`/`event_label`
 *  are optional context ("gas"/"B구역 가스 감지") the admin can attach; `instruction` is the
 *  actual free-text order and is the only field the server requires. */
@Serializable
data class InstructionDto(
    val id: Int? = null,
    val event_kind: String? = null,
    val event_label: String? = null,
    val instruction: String? = null,
    val created_at: String? = null
)

/** Maps the DB's Korean status string to the app's ZoneStatus. Unknown/missing values default
 *  to NORMAL rather than DANGER — a sensor that hasn't reported a recognizable status shouldn't
 *  silently trigger the full emergency alarm. */
fun statusFromKorean(raw: String?): ZoneStatus = when (raw?.trim()) {
    "위험" -> ZoneStatus.DANGER
    "주의", "경계" -> ZoneStatus.CAUTION
    "정상" -> ZoneStatus.NORMAL
    else -> ZoneStatus.NORMAL
}

/** Loose "is this alert_level dangerous" check for robot/fire alert_level strings, whose exact
 *  vocabulary isn't pinned down server-side yet (AI classifier output). Matches Korean "위험"
 *  or an English "danger"/"high", case-insensitively, anywhere in the string — deliberately
 *  permissive since under-triggering a safety alarm is worse than over-triggering one. */
fun isDangerLevel(level: String?): Boolean {
    if (level.isNullOrBlank()) return false
    val lower = level.lowercase()
    return level.contains("위험") || lower.contains("danger") || lower.contains("high")
}

/** "zone_A" -> "Zone A" (falls back to the raw id, capitalized, if it has no underscore). */
fun formatZoneName(zoneId: String): String {
    val suffix = zoneId.substringAfterLast('_', missingDelimiterValue = zoneId)
    return "Zone ${suffix.uppercase()}"
}
