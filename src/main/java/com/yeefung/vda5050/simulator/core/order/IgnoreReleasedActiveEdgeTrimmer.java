package com.yeefung.vda5050.simulator.core.order;

import com.fasterxml.jackson.databind.JsonNode;
import com.yeefung.vda5050.simulator.core.route.RouteFollower;
import com.yeefung.vda5050.simulator.map.Vda5050MapNameCodec;
import com.yeefung.vda5050.simulator.util.JsonNodes;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * In {@link com.yeefung.vda5050.simulator.config.OrderExecutionMode#IGNORE_RELEASED}, when a new MQTT
 * {@code order} arrives while the vehicle is still driving, drops leading edges that the vehicle has
 * already traversed so the route is not restarted from the first node (e.g. still between 2 and 3 after
 * an instantAction + new order that repeats 1→2→3).
 */
public final class IgnoreReleasedActiveEdgeTrimmer {

  private IgnoreReleasedActiveEdgeTrimmer() {}

  /**
   * @return edges to execute — same as input if nothing should be trimmed
   */
  public static List<JsonNode> trimIfDriving(
      List<JsonNode> activeEdges,
      List<JsonNode> sortedNodes,
      RouteFollower route,
      OrderRoutePlanner planner,
      Vda5050MapNameCodec codec
  ) {
    Objects.requireNonNull(activeEdges, "activeEdges");
    if (activeEdges.isEmpty() || route == null) {
      return activeEdges;
    }
    int skip = computeLeadingSkipCount(activeEdges, sortedNodes, route, planner, codec);
    if (skip <= 0) {
      return activeEdges;
    }
    if (skip >= activeEdges.size()) {
      return List.of();
    }
    return new ArrayList<>(activeEdges.subList(skip, activeEdges.size()));
  }

  /**
   * Prefer skipping {@code route.getCurrentSegmentIndex()} edges when the new order’s prefix matches the
   * already-driven route; otherwise find the first new-order edge that matches the current segment.
   */
  public static int computeLeadingSkipCount(
      List<JsonNode> activeEdges,
      List<JsonNode> sortedNodes,
      RouteFollower route,
      OrderRoutePlanner planner,
      Vda5050MapNameCodec codec
  ) {
    int segIdx = route.getCurrentSegmentIndex();
    int n = activeEdges.size();
    /*
     * If new order repeats edges we already finished (0 .. segIdx-1), skip those — even when
     * matching the *current* edge (segIdx) fails (node resolution quirks). Requiring
     * activeEdges[segIdx] to match caused skip=0 and a full rebuild from edge 0 → vehicle jumps back.
     */
    if (segIdx > 0) {
      boolean prefixOk = true;
      for (int j = 0; j < segIdx && j < n; j++) {
        if (!edgeMatchesRouteSegment(
            activeEdges.get(j), sortedNodes, planner, codec,
            route.getSegFrom(j), route.getSegTo(j))) {
          prefixOk = false;
          break;
        }
      }
      if (prefixOk) {
        return Math.min(segIdx, n);
      }
    }
    for (int i = 0; i < n; i++) {
      if (edgeMatchesRouteSegment(
          activeEdges.get(i), sortedNodes, planner, codec,
          route.getSegFrom(segIdx), route.getSegTo(segIdx))) {
        return i;
      }
    }
    return 0;
  }

  private static boolean edgeMatchesRouteSegment(
      JsonNode edge,
      List<JsonNode> sortedNodes,
      OrderRoutePlanner planner,
      Vda5050MapNameCodec codec,
      String routeFromOpenTcs,
      String routeToOpenTcs
  ) {
    String s = JsonNodes.text(edge, "startNodeId");
    String t = JsonNodes.text(edge, "endNodeId");
    if (s == null || t == null) {
      return false;
    }
    JsonNode ns = planner.findOrderNode(sortedNodes, s);
    JsonNode nt = planner.findOrderNode(sortedNodes, t);
    String nsId = ns != null ? JsonNodes.text(ns, "nodeId") : null;
    String ntId = nt != null ? JsonNodes.text(nt, "nodeId") : null;
    boolean fromOk =
        nsId != null && !nsId.isBlank()
            ? sameLogicalNode(nsId, routeFromOpenTcs, codec)
            : sameLogicalNode(s.trim(), routeFromOpenTcs, codec);
    boolean toOk =
        ntId != null && !ntId.isBlank()
            ? sameLogicalNode(ntId, routeToOpenTcs, codec)
            : sameLogicalNode(t.trim(), routeToOpenTcs, codec);
    return fromOk && toOk;
  }

  private static boolean sameLogicalNode(String a, String b, Vda5050MapNameCodec codec) {
    String va = codec.toVehicleNodeId(a.trim());
    String vb = codec.toVehicleNodeId(b.trim());
    return va.equals(vb);
  }
}
