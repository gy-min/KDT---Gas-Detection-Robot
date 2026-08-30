package com.gasrobot.monitor.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.gasrobot.monitor.data.model.ALL_ZONE_IDS
import com.gasrobot.monitor.data.model.EmergencyKind
import com.gasrobot.monitor.data.model.Zone
import com.gasrobot.monitor.ui.components.SectionCard
import com.gasrobot.monitor.ui.preview.PreviewData
import com.gasrobot.monitor.ui.theme.AppColors
import com.gasrobot.monitor.ui.viewmodel.MonitoringUiState
import java.time.LocalDateTime
import java.time.OffsetDateTime

/**
 * Landing screen for employees: current status at a glance (안전 / 위험) plus a compact
 * sensor grid mirroring the admin web's "SENSORS" panel. No 대피 경로/119 buttons here — those
 * already live in the bottom nav, so repeating them as cards was redundant. No robot fleet info
 * either; that moved to the separate web admin app. No ppm/농도 readout in the status card —
 * the per-zone numbers in the sensor grid below cover that in more useful, per-zone detail.
 *
 * Wrapped in verticalScroll as a safety net for smaller phones / larger system font sizes. With
 * the two removed action cards this should fit one screen without actually needing to scroll on
 * a typical device — but a plain Column with no scroll modifier *clips* instead of scrolling
 * when content ends up taller than the screen, which was the original bug (Zone C/D missing).
 */
@Composable
fun HomeScreen(
    state: MonitoringUiState,
    onLogout: () -> Unit = {}
) {
    val em = state.emergency.active

    Column(
        Modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(18.dp),
        verticalArrangement = Arrangement.spacedBy(14.dp)
    ) {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.End) {
            TextButton(onClick = onLogout) {
                Text("로그아웃", fontSize = 12.sp, color = AppColors.InkFaint)
            }
        }

        SectionCard(backgroundColor = if (em) AppColors.DangerBg else AppColors.SafeBg, padding = 20) {
            Text(
                if (em) "위험 상황" else "전 구역 정상",
                color = if (em) AppColors.DangerIcon else AppColors.SafeIcon,
                fontWeight = FontWeight.Bold, fontSize = 12.sp
            )
            Spacer(Modifier.height(6.dp))
            Text(
                if (em) (if (state.emergency.kind == EmergencyKind.FIRE) "화재 감지됨" else "가스 누출 감지됨") else "모든 구역 안전",
                color = if (em) AppColors.DangerText else AppColors.SafeText,
                fontWeight = FontWeight.Black, fontSize = 24.sp
            )
            Spacer(Modifier.height(8.dp))
            Text(
                if (em) state.emergency.riskSummary.ifBlank {
                    "${state.emergency.gasType} 감지 · ${state.emergency.zoneLabel}"
                } else "최근 스캔 이상 없음.",
                color = if (em) AppColors.DangerText else AppColors.SafeText,
                fontSize = 13.sp
            )
        }

        SensorGrid(state.zones)
    }
}

/** One gas-sensor zone (e.g. "A1", covering the TL-A edge on the map) plus its latest reading. */
private data class SensorPoint(val label: String, val zone: Zone?)

/**
 * Mobile version of the admin web's dark "SENSORS" sidebar panel — one card per zone
 * (`zone_A1`..`zone_C3`, see [ALL_ZONE_IDS]), each updating independently. 3 cards per row so
 * nothing wraps onto a half-empty line.
 */
@Composable
private fun SensorGrid(zones: List<Zone>) {
    if (zones.isEmpty()) return
    val byId = zones.associateBy { it.id }
    Column(verticalArrangement = Arrangement.spacedBy(10.dp)) {
        Text("SENSORS", color = AppColors.InkFaint, fontWeight = FontWeight.Bold, fontSize = 12.sp, letterSpacing = 1.sp)
        ALL_ZONE_IDS.chunked(3).forEach { rowIds ->
            Row(horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                rowIds.forEach { zoneId ->
                    val sensorPoint = SensorPoint(zoneId.removePrefix("zone_"), byId[zoneId])
                    SensorCard(sensorPoint, modifier = Modifier.weight(1f))
                }
            }
        }
    }
}

@Composable
private fun SensorCard(point: SensorPoint, modifier: Modifier = Modifier) {
    val status = point.zone?.status ?: com.gasrobot.monitor.data.model.ZoneStatus.NORMAL
    val cardBg = statusBg(status)
    val accent = statusAccent(status)
    val statusLabel = when (status) {
        com.gasrobot.monitor.data.model.ZoneStatus.NORMAL -> "정상"
        com.gasrobot.monitor.data.model.ZoneStatus.CAUTION -> "주의"
        com.gasrobot.monitor.data.model.ZoneStatus.DANGER -> "위험"
    }
    val intensity = point.zone?.intensity ?: 0
    val fraction = (intensity / 100f).coerceIn(0f, 1f)

    Row(modifier) {
        Box(Modifier.width(3.dp).fillMaxHeight().background(accent))
        Column(
            Modifier
                .weight(1f)
                .background(cardBg, RoundedCornerShape(topEnd = 10.dp, bottomEnd = 10.dp))
                .padding(8.dp)
        ) {
            Text(point.label, color = AppColors.Ink, fontWeight = FontWeight.Bold, fontSize = 13.sp)
            Text(statusLabel, color = accent, fontWeight = FontWeight.Bold, fontSize = 10.sp)
            Spacer(Modifier.height(4.dp))
            Row(verticalAlignment = Alignment.Bottom) {
                Text("$intensity", color = AppColors.Ink, fontWeight = FontWeight.Black, fontSize = 18.sp)
                Text(" /100", color = AppColors.InkFaint, fontSize = 10.sp, modifier = Modifier.padding(bottom = 2.dp))
            }
            Spacer(Modifier.height(4.dp))
            Box(
                Modifier
                    .fillMaxWidth()
                    .height(4.dp)
                    .clip(RoundedCornerShape(2.dp))
                    .background(Color.White.copy(alpha = 0.6f))
            ) {
                Box(
                    Modifier
                        .fillMaxWidth(fraction)
                        .fillMaxHeight()
                        .background(accent)
                )
            }
            Spacer(Modifier.height(4.dp))
            Text(formatUpdatedAt(point.zone?.updatedAt), color = AppColors.InkFaint, fontSize = 9.sp)
        }
    }
}

/** Same pastel tint the top status banner already uses per state, so the sensor grid reads as
 *  part of the same light UI instead of a dropped-in dark panel. */
private fun statusBg(status: com.gasrobot.monitor.data.model.ZoneStatus) = when (status) {
    com.gasrobot.monitor.data.model.ZoneStatus.NORMAL -> AppColors.SafeBg
    com.gasrobot.monitor.data.model.ZoneStatus.CAUTION -> AppColors.CautionFill
    com.gasrobot.monitor.data.model.ZoneStatus.DANGER -> AppColors.DangerBg
}

private fun statusAccent(status: com.gasrobot.monitor.data.model.ZoneStatus) = when (status) {
    com.gasrobot.monitor.data.model.ZoneStatus.NORMAL -> AppColors.SafeIcon
    com.gasrobot.monitor.data.model.ZoneStatus.CAUTION -> AppColors.CautionBorder
    com.gasrobot.monitor.data.model.ZoneStatus.DANGER -> AppColors.DangerIcon
}

/** Kept short (just "H:MM") since the sensor cards are now a third of the row's width — the
 *  old "갱신 14시 58분 0초" phrasing didn't fit without wrapping or clipping. */
private fun formatUpdatedAt(iso: String?): String {
    if (iso.isNullOrBlank()) return "정보 없음"
    val dt = runCatching { LocalDateTime.parse(iso) }
        .recoverCatching { OffsetDateTime.parse(iso).toLocalDateTime() }
        .getOrNull()
    return if (dt != null) "${dt.hour}:${dt.minute.toString().padStart(2, '0')}" else iso
}

@Preview(showBackground = true, name = "정상 상태")
@Composable
private fun HomeScreenSafePreview() {
    MaterialTheme { HomeScreen(PreviewData.safeUiState) }
}

@Preview(showBackground = true, name = "비상 상태")
@Composable
private fun HomeScreenEmergencyPreview() {
    MaterialTheme { HomeScreen(PreviewData.emergencyUiState) }
}
