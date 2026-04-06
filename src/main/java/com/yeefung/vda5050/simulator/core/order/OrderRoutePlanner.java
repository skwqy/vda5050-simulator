package com.yeefung.vda5050.simulator.core.order;

import com.fasterxml.jackson.databind.JsonNode;
import com.yeefung.vda5050.simulator.core.route.RouteFollower;
import com.yeefung.vda5050.simulator.map.LayoutTransform;
import com.yeefung.vda5050.simulator.map.MotionSegment;
import com.yeefung.vda5050.simulator.map.PathMotionFactory;
import com.yeefung.vda5050.simulator.map.PlantModel;
import com.yeefung.vda5050.simulator.map.PlantPath;
import com.yeefung.vda5050.simulator.map.PointMm;
import com.yeefung.vda5050.simulator.map.PolylineMotion;
import com.yeefung.vda5050.simulator.map.ReversedMotionSegment;
import com.yeefung.vda5050.simulator.map.Vda5050MapNameCodec;
import com.yeefung.vda5050.simulator.util.JsonNodes;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * Builds a {@link RouteFollower} from VDA order JSON: plant geometry when possible, else chords from
 * {@code nodePosition}, else a single straight leg between released endpoints.
 */
public final class OrderRoutePlanner {

  private static final System.Logger LOG = System.getLogger(OrderRoutePlanner.class.getName());

  private final PlantModel plantModel;
  private final LayoutTransform layoutTransform;
  private final PlantModel.NameResolver nameResolver;
  private final Vda5050MapNameCodec mapNameCodec;

  public OrderRoutePlanner(
      PlantModel plantModel,
      LayoutTransform layoutTransform,
      PlantModel.NameResolver nameResolver,
      Vda5050MapNameCodec mapNameCodec
  ) {
    this.plantModel = plantModel;
    this.layoutTransform = layoutTransform;
    this.nameResolver = nameResolver;
    this.mapNameCodec = mapNameCodec;
  }

  public Optional<RouteFollower> buildRoute(
      List<JsonNode> activeEdges,
      List<JsonNode> sortedNodes,
      JsonNode firstReleasedStartNode,
      JsonNode lastReleasedEndNode,
      double defaultSpeed
  ) {
    Optional<RouteFollower> built =
        plantModel != null
            ? buildRouteFromPlant(activeEdges, sortedNodes, defaultSpeed)
            : Optional.empty();
    if (built.isEmpty()) {
      built = buildRouteFromReleasedNodePositions(activeEdges, sortedNodes, defaultSpeed);
    }
    if (built.isEmpty()) {
      built =
          buildRouteSingleStraightReleased(
              activeEdges,
              firstReleasedStartNode,
              lastReleasedEndNode,
              defaultSpeed);
    }
    return built;
  }

  /**
   * For each released edge: use plant POLYPATH/BEZIER when resolvable; otherwise a straight chord
   * between {@code nodePosition}s from the order. Avoids dropping the whole route (and falling back
   * to all-chord or one diagonal) when a single edge fails to match the plant.
   */
  private Optional<RouteFollower> buildRouteFromPlant(
      List<JsonNode> releasedEdges,
      List<JsonNode> sortedNodes,
      double defaultSpeed
  ) {
    Map<String, JsonNode> byId = indexNodesById(sortedNodes);
    List<MotionSegment> segments = new ArrayList<>();
    List<String> segFrom = new ArrayList<>();
    List<String> segTo = new ArrayList<>();
    List<Long> segFromSeq = new ArrayList<>();
    List<Long> segToSeq = new ArrayList<>();
    List<Double> speeds = new ArrayList<>();

    for (JsonNode edge : releasedEdges) {
      String eid = JsonNodes.text(edge, "edgeId");
      String s0 = JsonNodes.text(edge, "startNodeId");
      String s1 = JsonNodes.text(edge, "endNodeId");
      if (s0 == null || s1 == null) {
        return Optional.empty();
      }
      double sp = JsonNodes.doubleOr(edge, "maxSpeed", defaultSpeed);
      Optional<PlantPath> pp = plantModel.resolvePath(eid, s0, s1, nameResolver);
      boolean reverseAlongPlant = false;
      if (pp.isEmpty()) {
        pp = plantModel.resolvePath("", s1, s0, nameResolver);
        reverseAlongPlant = pp.isPresent();
        if (reverseAlongPlant) {
          LOG.log(
              System.Logger.Level.INFO,
              "Using reverse plant path for order edge "
                  + s0
                  + " -> "
                  + s1
                  + " (map defines "
                  + s1
                  + " -> "
                  + s0
                  + ")"
          );
        }
      }
      boolean usedPlant = false;
      if (pp.isPresent()) {
        try {
          PlantPath path = pp.get();
          MotionSegment motion = PathMotionFactory.build(path, plantModel, layoutTransform);
          if (reverseAlongPlant) {
            motion = new ReversedMotionSegment(motion);
          }
          segments.add(motion);
          segFrom.add(mapNameCodec.toOpenTcsPointName(s0.trim()));
          segTo.add(mapNameCodec.toOpenTcsPointName(s1.trim()));
          segFromSeq.add(sequenceIdForOrderNode(sortedNodes, s0.trim()));
          segToSeq.add(sequenceIdForOrderNode(sortedNodes, s1.trim()));
          speeds.add(sp > 0 ? sp : defaultSpeed);
          usedPlant = true;
        } catch (RuntimeException ex) {
          LOG.log(
              System.Logger.Level.WARNING,
              "Plant geometry failed for edgeId=" + eid + ", using chord from order nodes",
              ex
          );
        }
      }
      if (!usedPlant) {
        if (pp.isEmpty()) {
          LOG.log(
              System.Logger.Level.WARNING,
              "No plant path for edgeId="
                  + eid
                  + " ("
                  + s0
                  + " -> "
                  + s1
                  + "); using straight chord from order nodePosition"
          );
        }
        JsonNode n0 = byId.get(s0.trim());
        if (n0 == null) {
          n0 = findOrderNode(sortedNodes, s0);
        }
        JsonNode n1 = byId.get(s1.trim());
        if (n1 == null) {
          n1 = findOrderNode(sortedNodes, s1);
        }
        if (n0 == null || n1 == null) {
          LOG.log(
              System.Logger.Level.WARNING,
              "Missing node in order for chord fallback: " + s0 + " / " + s1
          );
          return Optional.empty();
        }
        double[] a = nodePositionMeters(n0);
        double[] b = nodePositionMeters(n1);
        PointMm p0 = new PointMm(Math.round(a[0] * 1000.0), Math.round(a[1] * 1000.0));
        PointMm p1 = new PointMm(Math.round(b[0] * 1000.0), Math.round(b[1] * 1000.0));
        try {
          segments.add(new PolylineMotion(List.of(p0, p1)));
        } catch (RuntimeException ex) {
          return Optional.empty();
        }
        segFrom.add(mapNameCodec.toOpenTcsPointName(s0.trim()));
        segTo.add(mapNameCodec.toOpenTcsPointName(s1.trim()));
        segFromSeq.add(sequenceIdForOrderNode(sortedNodes, s0.trim()));
        segToSeq.add(sequenceIdForOrderNode(sortedNodes, s1.trim()));
        speeds.add(sp > 0 ? sp : defaultSpeed);
      }
    }

    return Optional.of(
        new RouteFollower(segments, segFrom, segTo, speeds, segFromSeq, segToSeq)
    );
  }

  private static Map<String, JsonNode> indexNodesById(List<JsonNode> sortedNodes) {
    Map<String, JsonNode> byId = new HashMap<>();
    for (JsonNode n : sortedNodes) {
      String id = JsonNodes.text(n, "nodeId");
      if (id != null && !id.isBlank()) {
        byId.putIfAbsent(id.trim(), n);
      }
    }
    return byId;
  }

  /**
   * Straight segments along each released edge using {@code nodePosition} from the order (metres).
   * Internal node ids are vehicle-side strings from the order.
   */
  private Optional<RouteFollower> buildRouteFromReleasedNodePositions(
      List<JsonNode> releasedEdges,
      List<JsonNode> sortedNodes,
      double defaultSpeed
  ) {
    Map<String, JsonNode> byId = indexNodesById(sortedNodes);
    List<MotionSegment> segments = new ArrayList<>();
    List<String> segFrom = new ArrayList<>();
    List<String> segTo = new ArrayList<>();
    List<Long> segFromSeq = new ArrayList<>();
    List<Long> segToSeq = new ArrayList<>();
    List<Double> speeds = new ArrayList<>();

    for (JsonNode edge : releasedEdges) {
      String s0 = JsonNodes.text(edge, "startNodeId");
      String s1 = JsonNodes.text(edge, "endNodeId");
      if (s0 == null || s1 == null) {
        return Optional.empty();
      }
      JsonNode n0 = byId.get(s0.trim());
      if (n0 == null) {
        n0 = findOrderNode(sortedNodes, s0);
      }
      JsonNode n1 = byId.get(s1.trim());
      if (n1 == null) {
        n1 = findOrderNode(sortedNodes, s1);
      }
      if (n0 == null || n1 == null) {
        return Optional.empty();
      }
      double[] a = nodePositionMeters(n0);
      double[] b = nodePositionMeters(n1);
      PointMm p0 = new PointMm(Math.round(a[0] * 1000.0), Math.round(a[1] * 1000.0));
      PointMm p1 = new PointMm(Math.round(b[0] * 1000.0), Math.round(b[1] * 1000.0));
      try {
        segments.add(new PolylineMotion(List.of(p0, p1)));
      } catch (RuntimeException ex) {
        return Optional.empty();
      }
      segFrom.add(s0.trim());
      segTo.add(s1.trim());
      segFromSeq.add(sequenceIdForOrderNode(sortedNodes, s0.trim()));
      segToSeq.add(sequenceIdForOrderNode(sortedNodes, s1.trim()));
      speeds.add(JsonNodes.doubleOr(edge, "maxSpeed", defaultSpeed));
    }

    return Optional.of(
        new RouteFollower(segments, segFrom, segTo, speeds, segFromSeq, segToSeq)
    );
  }

  /** One straight leg from first released start to last released end (weakest fallback). */
  private Optional<RouteFollower> buildRouteSingleStraightReleased(
      List<JsonNode> releasedEdges,
      JsonNode firstNode,
      JsonNode lastNode,
      double defaultSpeed
  ) {
    String fromId = JsonNodes.text(firstNode, "nodeId");
    String toId = JsonNodes.text(lastNode, "nodeId");
    if (fromId == null || toId == null) {
      return Optional.empty();
    }
    double[] start = nodePositionMeters(firstNode);
    double[] end = nodePositionMeters(lastNode);
    double sp = JsonNodes.doubleOr(releasedEdges.getFirst(), "maxSpeed", defaultSpeed);
    return Optional.of(
        RouteFollower.singleStraightLeg(
            fromId.trim(),
            toId.trim(),
            JsonNodes.longVal(firstNode, "sequenceId"),
            JsonNodes.longVal(lastNode, "sequenceId"),
            start[0],
            start[1],
            end[0],
            end[1],
            sp > 0 ? sp : defaultSpeed
        )
    );
  }

  /**
   * Resolves {@code nodes[]} for an edge endpoint reference. Edges may use stripped VDA {@code nodeId}
   * (e.g. {@code "2"}) while {@code nodes[]} uses OpenTCS names ({@code "Point_2"}) — must match
   * {@link Vda5050MapNameCodec} or we get wrong {@code sequenceId} and duplicate {@code state} at the same node.
   */
  public JsonNode findOrderNode(List<JsonNode> sortedNodes, String nodeIdRef) {
    if (nodeIdRef == null || nodeIdRef.isBlank()) {
      return null;
    }
    String w = nodeIdRef.trim();
    for (JsonNode n : sortedNodes) {
      String id = JsonNodes.text(n, "nodeId");
      if (id == null || id.isBlank()) {
        continue;
      }
      String idTrim = id.trim();
      if (idTrim.equals(w)) {
        return n;
      }
      if (idTrim.equals(mapNameCodec.toOpenTcsPointName(w))) {
        return n;
      }
      if (w.equals(mapNameCodec.toVehicleNodeId(idTrim))) {
        return n;
      }
    }
    return null;
  }

  private long sequenceIdForOrderNode(List<JsonNode> sortedNodes, String nodeIdRef) {
    JsonNode n = findOrderNode(sortedNodes, nodeIdRef);
    return n != null ? JsonNodes.longVal(n, "sequenceId") : 0L;
  }

  private static double[] nodePositionMeters(JsonNode node) {
    JsonNode np = node.get("nodePosition");
    if (np != null && np.isObject()) {
      Double px = JsonNodes.doubleVal(np, "x");
      Double py = JsonNodes.doubleVal(np, "y");
      if (px != null && py != null) {
        return new double[] {px, py};
      }
    }
    return new double[] {0.0, 0.0};
  }

  public static double maxEdgeSpeedMps(List<JsonNode> releasedEdges, double def) {
    double m = def;
    for (JsonNode e : releasedEdges) {
      Double ms = JsonNodes.doubleVal(e, "maxSpeed");
      if (ms != null && ms > 0) {
        m = ms;
        break;
      }
    }
    return m;
  }
}
