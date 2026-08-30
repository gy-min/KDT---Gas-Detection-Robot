package com.gasrobot.monitor.data.model

/**
 * Mirrors the real robot's `map_data.py` 1:1 — the same map the robot's own line-following
 * navigation and the server's `/api/evacuation-route` Dijkstra run both use, not a placeholder
 * floor plan. Coordinate system: (row, col), row 0..11 top-to-bottom, col 0..12 left-to-right.
 *
 * 12 named nodes: 4 exits (TL/TR/BL/BR), 2 dead-end waypoints (ML/MR — NOT exits), 6 rail
 * intersections (A..F). They form a lattice, not a simple row/column grid:
 *
 * ```
 * TL --- A --- B --- TR
 *        |     |
 * ML --- C --- D --- MR
 *        |     |
 * BL --- E --- F --- BR
 * ```
 *
 * The 9 gas-sensor zones (`zone_A1`..`zone_C3`, confirmed by SafeScout_API_계약서.md) each cover
 * exactly one horizontal edge — e.g. `zone_A2` is the A-B segment.
 *
 * EvacScreen shows the admin web's simple 3×3 zone-card grid, not a node diagram, so this file's
 * job is narrower than the full graph: give each zone card (a) a representative node id so a tap
 * can produce a valid `current_location` for `/api/evacuation-route` ([approxNodeForZone]), and
 * (b) a midpoint so the robot's raw coordinate can be matched to the nearest card
 * ([nearestZoneToRowCol]). `current_location` sent to the server is a node id; the route response
 * is a list of node ids (e.g. `["C","A","TL"]`), not coordinates.
 */
data class MapNode(val id: String, val row: Double, val col: Double)

val MAP_NODES: List<MapNode> = listOf(
    MapNode("TL", 1.0, 0.0), MapNode("TR", 0.0, 12.0),
    MapNode("BL", 11.0, 0.0), MapNode("BR", 10.0, 12.0),
    MapNode("ML", 5.0, 1.0), MapNode("MR", 5.0, 11.0),
    MapNode("A", 0.5, 4.0), MapNode("B", 0.5, 8.0),
    MapNode("C", 5.5, 4.0), MapNode("D", 5.5, 8.0),
    MapNode("E", 10.5, 4.0), MapNode("F", 10.5, 8.0),
)

private val nodesById = MAP_NODES.associateBy { it.id }

/** Zone code (no "zone_" prefix) -> the one edge (pair of node ids) it covers. Matches the
 *  server's `map_data.ZONES` exactly. */
val ZONE_EDGES: Map<String, Pair<String, String>> = mapOf(
    "A1" to ("TL" to "A"), "A2" to ("A" to "B"), "A3" to ("B" to "TR"),
    "B1" to ("ML" to "C"), "B2" to ("C" to "D"), "B3" to ("D" to "MR"),
    "C1" to ("BL" to "E"), "C2" to ("E" to "F"), "C3" to ("F" to "BR"),
)

/** All 9 zone_ids in a stable, grouped order — used by HomeScreen's per-zone sensor cards. */
val ALL_ZONE_IDS: List<String> = ZONE_EDGES.keys.map { "zone_$it" }

fun nodePosition(id: String): MapNode? = nodesById[id]

/** Parses a raw "(row,col)" QR coordinate string (ints or decimals — e.g. `robot_event.location`)
 *  into (row, col), for placing the robot pin at its exact spot on the map rather than snapping
 *  it to the nearest node. */
fun parseRowCol(label: String): Pair<Double, Double>? {
    val match = Regex("""^\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)$""").find(label.trim()) ?: return null
    val (row, col) = match.destructured
    return row.toDouble() to col.toDouble()
}

/** EvacScreen shows the admin web's simple 3×3 zone-card grid (A1..C3), not a node diagram —
 *  but `/api/evacuation-route`'s `current_location` needs one of the 12 node ids. A zone card
 *  is really one edge, not a single point, so this picks that edge's first node as "roughly
 *  where you'd be standing" if you tapped that card — an approximation, not a real position,
 *  since the server has no concept of "standing in a zone". Each of the 9 zones maps to a
 *  distinct node, so a tapped card and the node it produced can always be matched back up. */
fun approxNodeForZone(code: String): String? = ZONE_EDGES[code]?.first

/** Midpoint of a zone's edge, in (row, col) — used to find which zone card the robot's raw
 *  coordinate is closest to, so its pin is drawn on that card (see [nearestZoneToRowCol]). */
private fun zoneMidpoint(code: String): Pair<Double, Double>? {
    val (a, b) = ZONE_EDGES[code] ?: return null
    val na = nodePosition(a) ?: return null
    val nb = nodePosition(b) ?: return null
    return (na.row + nb.row) / 2.0 to (na.col + nb.col) / 2.0
}

/** Which zone card the robot's raw (row,col) position is closest to, within `maxDist` — null if
 *  nothing is close enough (e.g. the robot is off in Zone D territory that no longer exists). */
fun nearestZoneToRowCol(row: Double, col: Double, maxDist: Double = 4.0): String? {
    var bestCode: String? = null
    var bestDist = Double.MAX_VALUE
    for (code in ZONE_EDGES.keys) {
        val (r, c) = zoneMidpoint(code) ?: continue
        val dist = kotlin.math.hypot(r - row, c - col)
        if (dist < bestDist) {
            bestDist = dist
            bestCode = code
        }
    }
    return bestCode?.takeIf { bestDist <= maxDist }
}
