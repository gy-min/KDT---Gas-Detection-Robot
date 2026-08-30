package com.gasrobot.monitor.ui.screens

import androidx.compose.animation.core.LinearEasing
import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.tween
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateMapOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.scale
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.layout.LayoutCoordinates
import androidx.compose.ui.layout.onGloballyPositioned
import androidx.compose.ui.layout.positionInRoot
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.gasrobot.monitor.data.model.RobotMarker
import com.gasrobot.monitor.data.model.Zone
import com.gasrobot.monitor.data.model.ZoneStatus
import com.gasrobot.monitor.data.model.ZONE_EDGES
import com.gasrobot.monitor.data.model.approxNodeForZone
import com.gasrobot.monitor.data.model.nearestZoneToRowCol
import com.gasrobot.monitor.data.model.parseRowCol
import com.gasrobot.monitor.ui.components.SectionCard
import com.gasrobot.monitor.ui.preview.PreviewData
import com.gasrobot.monitor.ui.theme.AppColors
import com.gasrobot.monitor.ui.theme.ZoneMap
import com.gasrobot.monitor.ui.viewmodel.EvacRouteState
import com.gasrobot.monitor.ui.viewmodel.MonitoringUiState

/** The 4 corner exit nodes from `map_data.py` — the only route nodes with a direct visual
 *  marker on this card-grid map (see [ExitDoor]). */
private val EXIT_KEYS = setOf("TL", "TR", "BL", "BR")

/**
 * Employee-facing evacuation screen: the same 3×3 zone-card map the admin web shows (A1..C3,
 * colored by 정상/주의/위험), plus the recommended action, in one place.
 */
@Composable
fun EvacScreen(
    state: MonitoringUiState,
    evacRoute: EvacRouteState = EvacRouteState.Idle,
    onSelectLocation: (String) -> Unit = {}
) {
    val em = state.emergency.active
    val selectedNode = when (evacRoute) {
        is EvacRouteState.Idle -> null
        is EvacRouteState.Loading -> evacRoute.currentLocation
        is EvacRouteState.Loaded -> evacRoute.currentLocation
        is EvacRouteState.Failed -> evacRoute.currentLocation
    }
    // A zone card is really one map edge, not a single node — see approxNodeForZone. Reversing
    // that mapping here (rather than tracking "which card was tapped" separately) means the
    // highlighted card always agrees with whatever `current_location` was actually sent.
    val selectedZone = ZONE_EDGES.keys.firstOrNull { approxNodeForZone(it) == selectedNode }
    val routeNodes = (evacRoute as? EvacRouteState.Loaded)?.route ?: emptyList()
    val routeEdges = routeNodes.zipWithNext().map { (a, b) -> setOf(a, b) }.toSet()
    val routeZones = ZONE_EDGES.filterValues { (a, b) -> setOf(a, b) in routeEdges }.keys

    // Ordered list of things the route line actually passes through, on screen: the tapped
    // zone card (the true visual start — `routeNodes.first()` is often a junction node with no
    // card of its own), then each zone card the path crosses in order, then the exit door if the
    // route ends at one. Segments that cross a sensor-less vertical rail edge (no zone card)
    // just connect straight through to the next visible waypoint — there's nothing to draw a
    // dot on in between since this map doesn't render individual junction nodes.
    val routeWaypoints = buildList {
        selectedZone?.let { add(it) }
        for ((a, b) in routeNodes.zipWithNext()) {
            val code = ZONE_EDGES.entries.firstOrNull { (_, edge) -> edge == (a to b) || edge == (b to a) }?.key
            if (code != null && lastOrNull() != code) add(code)
        }
        val lastNode = routeNodes.lastOrNull()
        if (lastNode != null && lastNode in EXIT_KEYS && lastOrNull() != lastNode) add(lastNode)
    }

    // verticalScroll matters here specifically: the map panel is a fixed height, plus a title
    // and two text cards below it — on a lot of phones that's taller than the space left under
    // the bottom nav bar. A plain Column with no scroll modifier *clips* instead of scrolling
    // when that happens, which was the bug (구역 상태 box cut off).
    Column(
        Modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(14.dp)
    ) {
        Text("대피 경로", fontWeight = FontWeight.Black, fontSize = 20.sp)

        ZoneMapPanel(
            zones = state.zones,
            robotMarker = state.robotMarker,
            selectedZone = selectedZone,
            routeZones = routeZones,
            routeWaypoints = routeWaypoints,
            onTapZone = { code -> approxNodeForZone(code)?.let(onSelectLocation) }
        )

        SectionCard(backgroundColor = if (em) AppColors.DangerBg else Color(0xFFEEF4F8)) {
            Text(if (em) "위험 구역 감지됨" else "구역 상태", fontSize = 12.sp, fontWeight = FontWeight.Bold)
            Text(
                if (em) "이 구역을 피해서 이동하세요" else "봉쇄 구역 없음",
                fontSize = 16.sp, fontWeight = FontWeight.Black,
                color = if (em) AppColors.DangerText else AppColors.Ink
            )
            Text(
                state.emergency.recommendedAction.ifBlank {
                    if (em) "누출원 인근 통로 봉쇄 권고. 인원을 개방부로 유도하세요."
                    else "감지된 위험 구역이 없습니다."
                },
                fontSize = 12.sp, color = AppColors.InkFaint
            )
        }

        EvacRouteStatusCard(evacRoute)
    }
}

/** Route status readout under the map — tapping a zone card directly on [ZoneMapPanel] is the
 *  only way to set "current location" (see that composable's `onTapZone`), so this card is
 *  purely a read-out of whatever came back for the last tap. */
@Composable
private fun EvacRouteStatusCard(evacRoute: EvacRouteState) {
    SectionCard(backgroundColor = Color(0xFFEEF4F8)) {
        when (evacRoute) {
            is EvacRouteState.Idle -> {
                Text("경로 대기", fontSize = 12.sp, fontWeight = FontWeight.Bold)
                Text(
                    "위 지도를 탭해 현재 위치를 선택하면 최단 대피 경로를 계산해 지도 위에 표시합니다.",
                    fontSize = 12.sp, color = AppColors.InkFaint
                )
            }
            is EvacRouteState.Loading -> {
                Text("경로 계산 중…", fontSize = 12.sp, fontWeight = FontWeight.Bold)
            }
            is EvacRouteState.Loaded -> {
                Text(
                    "실시간 최적 경로 (${evacRoute.route.size}개 지점)",
                    fontSize = 12.sp, fontWeight = FontWeight.Bold, color = AppColors.SafeText
                )
                Text(evacRoute.route.joinToString(" → "), fontSize = 13.sp, fontWeight = FontWeight.Black)
                if (evacRoute.blockedZones.isNotEmpty()) {
                    Spacer(Modifier.height(6.dp))
                    Text(
                        "피한 구역: ${evacRoute.blockedZones.joinToString(", ")}",
                        fontSize = 11.sp, color = AppColors.InkFaint
                    )
                }
            }
            is EvacRouteState.Failed -> {
                Text("경로를 찾을 수 없습니다", fontSize = 12.sp, fontWeight = FontWeight.Bold, color = AppColors.DangerText)
                Text(evacRoute.message, fontSize = 12.sp, color = AppColors.InkFaint)
            }
        }
    }
}

/**
 * The dark "평면도 · 로봇 위치" map, matching the admin web's card-grid design 1:1:
 *  - A 3×3 grid of zone cards: one ROW per corridor (A/B/C top-to-bottom), point 1..3
 *    left-to-right within a row.
 *  - A short wall segment under each column between row A/B and row B/C.
 *  - Four exit doors: left/right of row A (beside A1/A3), left/right of row C (beside C1/C3).
 *    Row B has none.
 *  - A DANGER card gets a red hazard-stripe fill and a "폐쇄" badge instead of its normal
 *    content, and isn't tappable — same idea as before, just restyled to match the admin web.
 *
 * Each zone card really represents one edge of the real robot map (`map_data.py`), not a single
 * point — [approxNodeForZone] picks a representative node so a tap can still produce a valid
 * `current_location` for `/api/evacuation-route`. The robot pin is placed on whichever card's
 * edge midpoint is closest to `robot_event.location` (see [nearestZoneToRowCol]); if nothing is
 * close enough, it falls back to the caption line below the grid.
 *
 * `routeWaypoints` (zone codes / exit ids, in path order) is drawn as an actual line across the
 * map rather than just highlighted card borders: every card/door that reports its on-screen
 * position via `onGloballyPositioned` gets an entry in `waypointPositions`, and once every
 * waypoint in the current route has reported in, a line connects them in order (see
 * `RouteLineOverlay`). Positions are captured relative to the map box itself (`positionInRoot()`
 * minus the box's own root position) so the line lines up regardless of where this screen sits
 * in the rest of the layout.
 */
@Composable
private fun ZoneMapPanel(
    zones: List<Zone>,
    robotMarker: RobotMarker?,
    selectedZone: String?,
    routeZones: Set<String>,
    routeWaypoints: List<String>,
    onTapZone: (String) -> Unit
) {
    val zonesById = zones.associateBy { it.id }
    val robotZone = robotMarker?.locationLabel?.let { parseRowCol(it) }
        ?.let { (row, col) -> nearestZoneToRowCol(row, col) }

    // Absolute (root-window) positions, not positions relative to the map box — subtracting
    // `mapOrigin` happens at draw time in RouteLineOverlay instead of here. Storing already-
    // relative values here would risk baking in a stale `mapOrigin` (still Offset.Zero) from
    // whichever of these onGloballyPositioned callbacks happens to fire first; reading both
    // pieces of state fresh at draw time is self-correcting regardless of callback order.
    var mapOrigin by remember { mutableStateOf(Offset.Zero) }
    val waypointCenters = remember { mutableStateMapOf<String, Offset>() }
    fun reportPosition(key: String, coordinates: LayoutCoordinates) {
        waypointCenters[key] = coordinates.positionInRoot() +
            Offset(coordinates.size.width / 2f, coordinates.size.height / 2f)
    }

    Column(
        Modifier
            .fillMaxWidth()
            .background(ZoneMap.Background, MaterialTheme.shapes.large)
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween, verticalAlignment = Alignment.CenterVertically) {
            Text("평면도 · 로봇 위치", color = ZoneMap.TextPrimary, fontWeight = FontWeight.Bold, fontSize = 15.sp)
            Row(horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                LegendDot("정상", ZoneMap.Normal)
                LegendDot("주의", ZoneMap.Caution)
                LegendDot("위험", ZoneMap.Danger)
            }
        }
        Text("지도를 탭해 현재 위치를 선택하세요", color = ZoneMap.TextSecondary, fontSize = 11.sp)

        Box(
            Modifier
                .fillMaxWidth()
                .height(420.dp)
                .background(Color(0xFF080B09), MaterialTheme.shapes.medium)
                .onGloballyPositioned { mapOrigin = it.positionInRoot() }
        ) {
            GridBackdrop()

            Column(Modifier.fillMaxSize().padding(10.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                for (rowZone in 0..2) {
                    // Exit doors flank row A and row C only — row B has no exit on this floor plan.
                    val hasExit = rowZone != 1
                    val leftExitKey = if (rowZone == 0) "TL" else "BL"
                    val rightExitKey = if (rowZone == 0) "TR" else "BR"
                    Row(Modifier.weight(1f).fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                        if (hasExit) {
                            ExitDoor(
                                Modifier
                                    .weight(0.12f)
                                    .fillMaxHeight()
                                    .onGloballyPositioned { reportPosition(leftExitKey, it) }
                            )
                        } else {
                            Spacer(Modifier.weight(0.12f))
                        }

                        Row(Modifier.weight(0.76f).fillMaxHeight(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                            for (col in 0..2) {
                                val code = "${'A' + rowZone}${col + 1}"
                                val zone = zonesById["zone_$code"]
                                val blocked = zone?.status == ZoneStatus.DANGER
                                Box(
                                    Modifier
                                        .weight(1f)
                                        .fillMaxHeight()
                                        .onGloballyPositioned { reportPosition(code, it) }
                                ) {
                                    ZoneCell(
                                        label = code,
                                        zone = zone,
                                        isSelected = selectedZone == code,
                                        isOnRoute = code in routeZones,
                                        blocked = blocked,
                                        onClick = if (blocked) null else ({ onTapZone(code) }),
                                        modifier = Modifier.fillMaxSize()
                                    )
                                    if (robotZone == code && robotMarker != null) {
                                        RobotPin(robotMarker, Modifier.align(Alignment.BottomCenter).padding(bottom = 6.dp))
                                    }
                                }
                            }
                        }

                        if (hasExit) {
                            ExitDoor(
                                Modifier
                                    .weight(0.12f)
                                    .fillMaxHeight()
                                    .onGloballyPositioned { reportPosition(rightExitKey, it) }
                            )
                        } else {
                            Spacer(Modifier.weight(0.12f))
                        }
                    }
                    if (rowZone != 2) WallRow()
                }
            }

            RouteLineOverlay(routeWaypoints, waypointCenters, mapOrigin, Modifier.matchParentSize())
        }

        if (robotMarker != null && robotZone == null) {
            RobotStatusRow(robotMarker)
        }
    }
}

/** Draws the actual evacuation route as a line across the map, connecting `waypoints` (zone
 *  codes / exit ids, in path order) through whatever on-screen positions those cards/doors have
 *  reported so far, converted from absolute (root-window) coordinates to map-relative ones via
 *  `mapOrigin`. Draws nothing until every waypoint has reported in — a partially-drawn line from
 *  missing positions would be more confusing than no line for one frame. */
@Composable
private fun RouteLineOverlay(
    waypoints: List<String>,
    centers: Map<String, Offset>,
    mapOrigin: Offset,
    modifier: Modifier = Modifier
) {
    if (waypoints.size < 2) return
    val points = waypoints.mapNotNull { centers[it]?.minus(mapOrigin) }
    if (points.size != waypoints.size) return

    Canvas(modifier) {
        for (i in 0 until points.size - 1) {
            drawLine(
                AppColors.Primary,
                points[i], points[i + 1],
                strokeWidth = 5.dp.toPx(),
                cap = StrokeCap.Round
            )
        }
        points.forEach { p -> drawCircle(AppColors.Primary, radius = 5.dp.toPx(), center = p) }
    }
}

@Composable
private fun LegendDot(label: String, color: Color) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        Box(Modifier.size(8.dp).background(color, MaterialTheme.shapes.extraSmall))
        Spacer(Modifier.width(4.dp))
        Text(label, color = ZoneMap.TextSecondary, fontSize = 11.sp)
    }
}

/** Three short wall segments, one under each corridor column. Spacers on both sides keep the
 *  bars aligned under the cell columns, not the exit-door slots those columns sit between
 *  (mirrors the 0.12f/0.76f/0.12f split each row uses). */
@Composable
private fun WallRow() {
    Row(Modifier.fillMaxWidth().height(14.dp), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
        Spacer(Modifier.weight(0.12f))
        Row(Modifier.weight(0.76f).fillMaxHeight(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            repeat(3) {
                Box(
                    Modifier
                        .weight(1f)
                        .fillMaxHeight()
                        .background(ZoneMap.CorridorBar.copy(alpha = 0.55f), MaterialTheme.shapes.extraSmall)
                )
            }
        }
        Spacer(Modifier.weight(0.12f))
    }
}

/** One zone card (e.g. "A1"). `zone` is nullable because a reading may not have arrived yet
 *  (cold start before the first REST/WebSocket update) — renders as an empty/grey placeholder
 *  rather than crashing or guessing a status. A blocked (DANGER) card shows hazard stripes and a
 *  "폐쇄" badge instead of its normal content, and `onClick` is null — matching the admin web,
 *  where a closed zone can't be picked as your current location. */
@Composable
private fun ZoneCell(
    label: String,
    zone: Zone?,
    isSelected: Boolean,
    isOnRoute: Boolean,
    blocked: Boolean,
    onClick: (() -> Unit)?,
    modifier: Modifier = Modifier
) {
    val status = zone?.status ?: ZoneStatus.NORMAL
    val statusColor = AppColors.zoneColor(status)
    val statusLabel = when (status) {
        ZoneStatus.NORMAL -> "정상"
        ZoneStatus.CAUTION -> "주의"
        ZoneStatus.DANGER -> "위험"
    }
    val borderColor = when {
        isSelected -> AppColors.Primary
        isOnRoute -> ZoneMap.Normal
        zone != null -> statusColor
        else -> ZoneMap.GridLine
    }
    val borderWidth = if (isSelected || isOnRoute) 2.5.dp else 1.5.dp

    Box(modifier) {
        Box(
            Modifier
                .fillMaxSize()
                .background(ZoneMap.fillFor(status), MaterialTheme.shapes.medium)
                .border(BorderStroke(borderWidth, borderColor), MaterialTheme.shapes.medium)
                .then(if (onClick != null) Modifier.clickable(onClick = onClick) else Modifier)
        ) {
            if (blocked) HazardStripes(Modifier.matchParentSize())
            Column(Modifier.padding(10.dp)) {
                Text(label, color = ZoneMap.TextPrimary, fontWeight = FontWeight.Bold, fontSize = 14.sp)
                if (zone != null) {
                    Text(
                        "${"%.1f".format(zone.intensity.toDouble())} / 100 · $statusLabel",
                        color = statusColor, fontSize = 11.sp, fontWeight = FontWeight.SemiBold
                    )
                }
            }
        }
        if (blocked) ClosedBadge(Modifier.align(Alignment.TopEnd).offset(x = 4.dp, y = (-4).dp))
    }
}

/** Diagonal hazard-stripe fill for a blocked cell — matches the admin web's "폐쇄" styling. */
@Composable
private fun HazardStripes(modifier: Modifier = Modifier) {
    Canvas(modifier) {
        val stripeWidth = 7.dp.toPx()
        val period = 16.dp.toPx()
        rotate(45f) {
            var x = -size.height
            while (x < size.width + size.height) {
                drawLine(
                    Color.Black.copy(alpha = 0.25f),
                    Offset(x, -size.height),
                    Offset(x, size.height * 2),
                    strokeWidth = stripeWidth
                )
                x += period
            }
        }
    }
}

/** Small "폐쇄"(closed) pill, pinned to a blocked card's top-right corner. */
@Composable
private fun ClosedBadge(modifier: Modifier = Modifier) {
    Row(
        modifier
            .background(ZoneMap.Danger, RoundedCornerShape(50))
            .padding(horizontal = 8.dp, vertical = 3.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text("－", color = Color.White, fontSize = 10.sp, fontWeight = FontWeight.Bold)
        Spacer(Modifier.width(3.dp))
        Text("폐쇄", color = Color.White, fontSize = 10.sp, fontWeight = FontWeight.Bold)
    }
}

/** Pulsing ring pinned directly on the zone card the robot is currently closest to, with a
 *  small status label underneath — matches the admin web showing the robot inline on the map. */
@Composable
private fun RobotPin(robot: RobotMarker, modifier: Modifier = Modifier) {
    val transition = rememberInfiniteTransition(label = "robotPulse")
    val pulse by transition.animateFloat(
        initialValue = 0.85f,
        targetValue = 1.35f,
        animationSpec = infiniteRepeatable(tween(1200, easing = LinearEasing), RepeatMode.Reverse),
        label = "pulse"
    )

    Column(modifier, horizontalAlignment = Alignment.CenterHorizontally) {
        Box(contentAlignment = Alignment.Center) {
            Box(
                Modifier
                    .size(24.dp)
                    .scale(pulse)
                    .background(ZoneMap.Caution.copy(alpha = 0.35f), MaterialTheme.shapes.extraLarge)
            )
            Box(
                Modifier
                    .size(16.dp)
                    .background(Color(0xFF2A2F2C), MaterialTheme.shapes.extraLarge)
                    .border(BorderStroke(1.5.dp, ZoneMap.Caution), MaterialTheme.shapes.extraLarge)
            )
        }
        Spacer(Modifier.height(2.dp))
        Text(
            robot.statusLabel,
            color = ZoneMap.TextPrimary, fontSize = 9.sp, fontWeight = FontWeight.Bold
        )
    }
}

/** Static "출구" door marker filling one row's exit slot (row A or row C, left or right side) —
 *  matching the admin web. */
@Composable
private fun ExitDoor(modifier: Modifier = Modifier) {
    Box(
        modifier
            .fillMaxSize()
            .background(Color(0xFF1A1512), MaterialTheme.shapes.small)
            .border(BorderStroke(1.5.dp, ZoneMap.Caution), MaterialTheme.shapes.small),
        contentAlignment = Alignment.Center
    ) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Text("출", color = ZoneMap.Caution, fontSize = 11.sp, fontWeight = FontWeight.Bold)
            Text("구", color = ZoneMap.Caution, fontSize = 11.sp, fontWeight = FontWeight.Bold)
        }
    }
}

/** Pulsing dot + "(row,col) · 상태" caption, shown under the map only when the robot's location
 *  doesn't resolve to any zone card. */
@Composable
private fun RobotStatusRow(robot: RobotMarker) {
    val transition = rememberInfiniteTransition(label = "robotPulse")
    val pulse by transition.animateFloat(
        initialValue = 0.85f,
        targetValue = 1.35f,
        animationSpec = infiniteRepeatable(tween(1200, easing = LinearEasing), RepeatMode.Reverse),
        label = "pulse"
    )

    Row(verticalAlignment = Alignment.CenterVertically) {
        Box(contentAlignment = Alignment.Center) {
            Box(
                Modifier
                    .size(18.dp)
                    .scale(pulse)
                    .background(ZoneMap.TextSecondary.copy(alpha = 0.25f), MaterialTheme.shapes.extraLarge)
            )
            Box(
                Modifier
                    .size(11.dp)
                    .background(Color(0xFF2A2F2C), MaterialTheme.shapes.extraLarge)
                    .border(BorderStroke(1.dp, ZoneMap.TextSecondary), MaterialTheme.shapes.extraLarge)
            )
        }
        Spacer(Modifier.width(8.dp))
        Text(
            "최근 로봇 위치 ${robot.locationLabel} · ${robot.statusLabel}",
            color = ZoneMap.TextSecondary, fontSize = 12.sp, fontWeight = FontWeight.Medium
        )
    }
}

/** Faint grid lines behind the zone cards, matching the admin web's graph-paper background. */
@Composable
private fun GridBackdrop() {
    Canvas(Modifier.fillMaxSize()) {
        val step = 28.dp.toPx()
        var x = 0f
        while (x < size.width) {
            drawLine(ZoneMap.GridLine, Offset(x, 0f), Offset(x, size.height))
            x += step
        }
        var y = 0f
        while (y < size.height) {
            drawLine(ZoneMap.GridLine, Offset(0f, y), Offset(size.width, y))
            y += step
        }
    }
}

@Preview(showBackground = true, name = "정상 상태")
@Composable
private fun EvacScreenSafePreview() {
    MaterialTheme { EvacScreen(PreviewData.safeUiState) }
}

@Preview(showBackground = true, name = "비상 상태")
@Composable
private fun EvacScreenEmergencyPreview() {
    MaterialTheme { EvacScreen(PreviewData.emergencyUiState) }
}
