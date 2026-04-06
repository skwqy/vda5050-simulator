package com.yeefung.vda5050.simulator.core;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.JsonNodeFactory;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.yeefung.vda5050.simulator.config.SimulatorProperties;
import com.yeefung.vda5050.simulator.map.LayoutTransform;
import com.yeefung.vda5050.simulator.map.MotionSegment;
import com.yeefung.vda5050.simulator.map.PathMotionFactory;
import com.yeefung.vda5050.simulator.map.PlantModel;
import com.yeefung.vda5050.simulator.map.PlantPath;
import com.yeefung.vda5050.simulator.map.PointMm;
import com.yeefung.vda5050.simulator.map.PolylineMotion;
import com.yeefung.vda5050.simulator.map.ReversedMotionSegment;
import com.yeefung.vda5050.simulator.map.Vda5050MapNameCodec;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Queue;

/**
 * Movement compatible with YFAOS {@code MessageResponseMatcher#orderAccepted}: {@code lastNodeId}
 * must reach the segment destination. When a plant XML is loaded, motion follows POLYPATH / BEZIER
 * geometry like {@code AgvCommunicationAdapter}.
 */
public final class SimulationEngine {

  private static final System.Logger LOG = System.getLogger(SimulationEngine.class.getName());

  private final SimulatorProperties props;
  private final PlantModel plantModel;
  private final LayoutTransform layoutTransform;
  private final PlantModel.NameResolver nameResolver;
  private final Vda5050MapNameCodec mapNameCodec;
  private final JsonNodeFactory json = JsonNodeFactory.instance;

  private double x;
  private double y;
  private double theta;
  private String mapId = "";

  private String lastNodeId = "";
  private long lastNodeSequenceId;

  private String orderId = "";
  private long orderUpdateId;

  private boolean driving;
  private RouteFollower route;

  private final Queue<InstantActionPending> instantQueue = new ArrayDeque<>();
  private final List<ObjectNode> lastActionStates = new ArrayList<>();

  private boolean statePublishedOnce;
  private String lastPublishedOpenTcsNodeId = "";
  private long lastPublishedNodeSequenceId;

  public SimulationEngine(SimulatorProperties props) {
    this(props, null);
  }

  public SimulationEngine(SimulatorProperties props, PlantModel plantModel) {
    this.props = Objects.requireNonNull(props, "props");
    this.plantModel = plantModel;
    this.layoutTransform = plantModel != null ? new LayoutTransform(props.map) : null;
    this.mapNameCodec =
        new Vda5050MapNameCodec(
            props.map.applyStripping, props.map.pointPrefix, props.map.pathPrefix);
    this.nameResolver =
        new PlantModel.NameResolver(
            props.map.applyStripping, props.map.pointPrefix, props.map.pathPrefix);
    if (props.map.mapId != null && !props.map.mapId.isBlank()) {
      this.mapId = props.map.mapId.trim();
    } else if (plantModel != null) {
      String mn = plantModel.getModelName();
      this.mapId = mn != null && !mn.isBlank() ? mn : "";
    }
    applyInitialPoseFromPlant();
  }

  /**
   * Places the vehicle on {@link SimulatorProperties.Simulation#initialPointName} using coordinates
   * from the plant XML when available. Always sets {@code lastNodeId} for the first {@code state}
   * when {@code initialPointName} is non-blank, even without a loaded plant or missing point in XML.
   */
  private void applyInitialPoseFromPlant() {
    String want = props.simulation.initialPointName;
    if (want == null || want.isBlank()) {
      return;
    }
    String w = want.trim();
    String canonicalForReport = mapNameCodec.toOpenTcsPointName(w);
    if (canonicalForReport == null || canonicalForReport.isBlank()) {
      canonicalForReport = w;
    }

    if (plantModel == null) {
      lastNodeId = canonicalForReport;
      lastNodeSequenceId = 0L;
      System.getLogger(SimulationEngine.class.getName()).log(
          System.Logger.Level.INFO,
          "initialPointName=\"" + w + "\" — no plant XML; lastNodeId set for state, pose at (0,0)"
      );
      return;
    }

    LinkedHashSet<String> candidates = new LinkedHashSet<>();
    candidates.add(w);
    candidates.add(canonicalForReport);
    for (String k : nameResolver.pointKeys(w)) {
      candidates.add(k);
    }
    for (String key : candidates) {
      if (key == null || key.isBlank()) {
        continue;
      }
      Optional<PointMm> opt = plantModel.point(key);
      if (opt.isEmpty()) {
        continue;
      }
      PointMm pm = opt.get();
      x = pm.xMeters();
      y = pm.yMeters();
      theta = 0.0;
      lastNodeId = key;
      lastNodeSequenceId = 0L;
      return;
    }
    lastNodeId = canonicalForReport;
    lastNodeSequenceId = 0L;
    System.getLogger(SimulationEngine.class.getName()).log(
        System.Logger.Level.WARNING,
        "initialPointName=\"" + w + "\" not found in plant; lastNodeId still set for state, pose at (0,0)"
    );
  }

  public synchronized void setMapId(String mapId) {
    if (mapId != null && !mapId.isBlank()) {
      this.mapId = mapId;
    }
  }

  /**
   * Whether to emit a {@code state} message: on first publish, or when the vehicle reaches a new
   * point ({@code lastNodeId} / {@code lastNodeSequenceId} change). Instant-action completions are
   * included in the next such state (no periodic state).
   */
  public synchronized boolean shouldPublishState() {
    flushInstantToActionStates();
    if (!statePublishedOnce) {
      return true;
    }
    return !Objects.equals(lastNodeId, lastPublishedOpenTcsNodeId)
        || lastNodeSequenceId != lastPublishedNodeSequenceId;
  }

  /** Call after a {@code state} payload was successfully sent. */
  public synchronized void afterStatePublished() {
    statePublishedOnce = true;
    lastPublishedOpenTcsNodeId = lastNodeId;
    lastPublishedNodeSequenceId = lastNodeSequenceId;
  }

  public synchronized void onOrderMessage(JsonNode root) {
    String oid = text(root, "orderId");
    Long ouid = longVal(root, "orderUpdateId");
    JsonNode nodes = root.get("nodes");
    if (nodes == null || !nodes.isArray() || nodes.isEmpty()) {
      return;
    }
    List<JsonNode> sortedNodes = new ArrayList<>();
    nodes.forEach(sortedNodes::add);
    sortedNodes.sort(Comparator.comparingLong(n -> longVal(n, "sequenceId")));

    orderId = oid != null ? oid : "";
    orderUpdateId = ouid != null ? ouid : 0L;

    List<JsonNode> releasedEdges = collectReleasedEdges(root);
    if (releasedEdges.isEmpty()) {
      System.getLogger(SimulationEngine.class.getName()).log(
          System.Logger.Level.WARNING,
          "order has no released edges — skip motion (horizon-only order)");
      route = null;
      driving = false;
      return;
    }

    double speed = maxEdgeSpeedMps(releasedEdges, props.simulation.defaultSpeedMps);

    JsonNode firstReleasedStartNode =
        findNodeById(sortedNodes, text(releasedEdges.getFirst(), "startNodeId"));
    JsonNode lastReleasedEndNode =
        findNodeById(sortedNodes, text(releasedEdges.getLast(), "endNodeId"));
    if (firstReleasedStartNode == null || lastReleasedEndNode == null) {
      System.getLogger(SimulationEngine.class.getName()).log(
          System.Logger.Level.WARNING,
          "order nodes missing for released edge endpoints");
      route = null;
      driving = false;
      return;
    }

    Optional<RouteFollower> built =
        plantModel != null ? buildRouteFromPlant(releasedEdges, sortedNodes, speed) : Optional.empty();
    if (built.isEmpty()) {
      built = buildRouteFromReleasedNodePositions(releasedEdges, sortedNodes, speed);
    }
    if (built.isEmpty()) {
      built =
          buildRouteSingleStraightReleased(
              releasedEdges,
              firstReleasedStartNode,
              lastReleasedEndNode,
              speed);
    }
    if (built.isEmpty()) {
      route = null;
      driving = false;
      return;
    }

    route = built.get();
    route.start();
    x = route.x;
    y = route.y;
    theta = route.theta;
    lastNodeId = route.lastNodeId;
    lastNodeSequenceId = longVal(firstReleasedStartNode, "sequenceId");
    driving = true;
  }

  private static List<JsonNode> collectReleasedEdges(JsonNode orderRoot) {
    JsonNode edgesNode = orderRoot.get("edges");
    if (edgesNode == null || !edgesNode.isArray()) {
      return List.of();
    }
    List<JsonNode> out = new ArrayList<>();
    for (JsonNode e : edgesNode) {
      if (edgeReleased(e)) {
        out.add(e);
      }
    }
    out.sort(Comparator.comparingLong(e -> longVal(e, "sequenceId")));
    return out;
  }

  /** VDA: {@code false} = horizon (not driven). {@code null} treated as released for lenient parsing. */
  private static boolean edgeReleased(JsonNode edge) {
    JsonNode r = edge.get("released");
    if (r == null || r.isNull()) {
      return true;
    }
    return r.asBoolean();
  }

  private static JsonNode findNodeById(List<JsonNode> sortedNodes, String nodeId) {
    if (nodeId == null || nodeId.isBlank()) {
      return null;
    }
    String want = nodeId.trim();
    for (JsonNode n : sortedNodes) {
      String id = text(n, "nodeId");
      if (id != null && id.trim().equals(want)) {
        return n;
      }
    }
    return null;
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
    List<Double> speeds = new ArrayList<>();

    for (JsonNode edge : releasedEdges) {
      String eid = text(edge, "edgeId");
      String s0 = text(edge, "startNodeId");
      String s1 = text(edge, "endNodeId");
      if (s0 == null || s1 == null) {
        return Optional.empty();
      }
      double sp = doubleOr(edge, "maxSpeed", defaultSpeed);
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
        JsonNode n1 = byId.get(s1.trim());
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
          segments.add(new PolylineMotion(java.util.List.of(p0, p1)));
        } catch (RuntimeException ex) {
          return Optional.empty();
        }
        segFrom.add(mapNameCodec.toOpenTcsPointName(s0.trim()));
        segTo.add(mapNameCodec.toOpenTcsPointName(s1.trim()));
        speeds.add(sp > 0 ? sp : defaultSpeed);
      }
    }

    JsonNode firstEdge = releasedEdges.getFirst();
    JsonNode lastEdge = releasedEdges.getLast();
    long firstSeq = sequenceIdForNodeId(sortedNodes, text(firstEdge, "startNodeId"));
    long lastSeq = sequenceIdForNodeId(sortedNodes, text(lastEdge, "endNodeId"));
    return Optional.of(
        new RouteFollower(
            segments,
            segFrom,
            segTo,
            speeds,
            firstSeq,
            lastSeq
        )
    );
  }

  private static Map<String, JsonNode> indexNodesById(List<JsonNode> sortedNodes) {
    Map<String, JsonNode> byId = new HashMap<>();
    for (JsonNode n : sortedNodes) {
      String id = text(n, "nodeId");
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
    List<Double> speeds = new ArrayList<>();

    for (JsonNode edge : releasedEdges) {
      String s0 = text(edge, "startNodeId");
      String s1 = text(edge, "endNodeId");
      if (s0 == null || s1 == null) {
        return Optional.empty();
      }
      JsonNode n0 = byId.get(s0.trim());
      JsonNode n1 = byId.get(s1.trim());
      if (n0 == null || n1 == null) {
        return Optional.empty();
      }
      double[] a = nodePositionMeters(n0);
      double[] b = nodePositionMeters(n1);
      PointMm p0 = new PointMm(Math.round(a[0] * 1000.0), Math.round(a[1] * 1000.0));
      PointMm p1 = new PointMm(Math.round(b[0] * 1000.0), Math.round(b[1] * 1000.0));
      try {
        segments.add(new PolylineMotion(java.util.List.of(p0, p1)));
      } catch (RuntimeException ex) {
        return Optional.empty();
      }
      segFrom.add(s0.trim());
      segTo.add(s1.trim());
      speeds.add(doubleOr(edge, "maxSpeed", defaultSpeed));
    }

    JsonNode fe = releasedEdges.getFirst();
    JsonNode le = releasedEdges.getLast();
    long firstSeq = sequenceIdForNodeId(sortedNodes, text(fe, "startNodeId"));
    long lastSeq = sequenceIdForNodeId(sortedNodes, text(le, "endNodeId"));
    return Optional.of(
        new RouteFollower(segments, segFrom, segTo, speeds, firstSeq, lastSeq)
    );
  }

  /** One straight leg from first released start to last released end (weakest fallback). */
  private Optional<RouteFollower> buildRouteSingleStraightReleased(
      List<JsonNode> releasedEdges,
      JsonNode firstNode,
      JsonNode lastNode,
      double defaultSpeed
  ) {
    String fromId = text(firstNode, "nodeId");
    String toId = text(lastNode, "nodeId");
    if (fromId == null || toId == null) {
      return Optional.empty();
    }
    double[] start = nodePositionMeters(firstNode);
    double[] end = nodePositionMeters(lastNode);
    double sp = doubleOr(releasedEdges.getFirst(), "maxSpeed", defaultSpeed);
    return Optional.of(
        RouteFollower.singleStraightLeg(
            fromId.trim(),
            toId.trim(),
            longVal(firstNode, "sequenceId"),
            longVal(lastNode, "sequenceId"),
            start[0],
            start[1],
            end[0],
            end[1],
            sp > 0 ? sp : defaultSpeed
        )
    );
  }

  private static long sequenceIdForNodeId(List<JsonNode> sortedNodes, String nodeId) {
    JsonNode n = findNodeById(sortedNodes, nodeId);
    return n != null ? longVal(n, "sequenceId") : 0L;
  }

  private static double[] nodePositionMeters(JsonNode node) {
    JsonNode np = node.get("nodePosition");
    if (np != null && np.isObject()) {
      Double px = doubleVal(np, "x");
      Double py = doubleVal(np, "y");
      if (px != null && py != null) {
        return new double[] {px, py};
      }
    }
    return new double[] {0.0, 0.0};
  }

  private static double maxEdgeSpeedMps(List<JsonNode> releasedEdges, double def) {
    double m = def;
    for (JsonNode e : releasedEdges) {
      Double ms = doubleVal(e, "maxSpeed");
      if (ms != null && ms > 0) {
        m = ms;
        break;
      }
    }
    return m;
  }

  private double doubleOr(JsonNode e, String field, double def) {
    Double v = doubleVal(e, field);
    return v != null && v > 0 ? v : def;
  }

  public synchronized void onInstantActionsMessage(JsonNode root) {
    JsonNode actions = root.get("actions");
    if (actions == null || !actions.isArray()) {
      return;
    }
    for (JsonNode a : actions) {
      String type = text(a, "actionType");
      String aid = text(a, "actionId");
      if (aid == null || aid.isEmpty()) {
        continue;
      }
      if (type != null && type.equalsIgnoreCase("cancelOrder")) {
        route = null;
        driving = false;
        orderId = "";
        orderUpdateId = 0;
        lastActionStates.clear();
        instantQueue.clear();
        lastActionStates.add(actionFinished(aid, type));
        return;
      }
      instantQueue.add(new InstantActionPending(aid, type != null ? type : "instant"));
    }
  }

  public synchronized void tick(double dtSeconds) {
    flushInstantToActionStates();

    if (route == null) {
      driving = false;
      return;
    }

    boolean done = route.advance(dtSeconds);
    x = route.x;
    y = route.y;
    theta = route.theta;
    lastNodeId = route.lastNodeId;
    lastNodeSequenceId = route.lastNodeSequenceId;
    driving = !done;
    if (done) {
      route = null;
    }
  }

  private void flushInstantToActionStates() {
    while (!instantQueue.isEmpty()) {
      InstantActionPending p = instantQueue.poll();
      lastActionStates.add(actionFinished(p.actionId, p.actionType));
    }
  }

  private ObjectNode actionFinished(String actionId, String actionType) {
    ObjectNode o = json.objectNode();
    o.put("actionId", actionId);
    o.put("actionType", actionType);
    o.put("actionStatus", "FINISHED");
    return o;
  }

  public synchronized ObjectNode buildState(HeaderIds ids) {
    flushInstantToActionStates();
    ArrayNode actionStates = json.arrayNode();
    for (ObjectNode a : lastActionStates) {
      actionStates.add(a);
    }
    lastActionStates.clear();

    ObjectNode root = json.objectNode();
    root.put("headerId", ids.headerId);
    root.put("timestamp", java.time.Instant.now().toString());
    root.put("version", props.simulation.protocolVersion);
    root.put("manufacturer", props.mqtt.manufacturer);
    root.put("serialNumber", props.mqtt.serialNumber);

    root.put("orderId", orderId);
    root.put("orderUpdateId", orderUpdateId);
    root.put(
        "lastNodeId",
        lastNodeId.isEmpty() ? "" : mapNameCodec.toVehicleNodeId(lastNodeId));
    root.put("lastNodeSequenceId", lastNodeSequenceId);
    root.set("nodeStates", json.arrayNode());
    root.set("edgeStates", json.arrayNode());
    root.put("driving", driving);
    root.set("actionStates", actionStates);

    ObjectNode battery = json.objectNode();
    battery.put("batteryCharge", 100.0);
    battery.put("charging", false);
    root.set("batteryState", battery);

    root.put("operatingMode", props.simulation.operatingMode);

    root.set("errors", json.arrayNode());

    ObjectNode safety = json.objectNode();
    safety.put("eStop", "NONE");
    safety.put("fieldViolation", false);
    root.set("safetyState", safety);

    ObjectNode pos = json.objectNode();
    pos.put("x", x);
    pos.put("y", y);
    pos.put("theta", theta);
    // AOS AgvPosition / JSON schema require mapId on every state message
    pos.put("mapId", effectiveMapIdForJson());
    pos.put("positionInitialized", true);
    root.set("agvPosition", pos);

    if (!orderId.isEmpty()) {
      root.put("orderState", driving ? "ACTIVE" : "ACTIVE");
    }

    return root;
  }

  public synchronized ObjectNode buildVisualization(HeaderIds ids) {
    ObjectNode root = json.objectNode();
    root.put("headerId", ids.headerId);
    root.put("timestamp", java.time.Instant.now().toString());
    root.put("version", props.simulation.protocolVersion);
    root.put("manufacturer", props.mqtt.manufacturer);
    root.put("serialNumber", props.mqtt.serialNumber);
    ObjectNode pos = json.objectNode();
    pos.put("x", x);
    pos.put("y", y);
    pos.put("theta", theta);
    pos.put("mapId", effectiveMapIdForJson());
    pos.put("positionInitialized", true);
    root.set("agvPosition", pos);
    return root;
  }

  /** VDA / AOS schema: {@code agvPosition.mapId} is required; never omit or send blank. */
  private String effectiveMapIdForJson() {
    if (mapId != null && !mapId.isBlank()) {
      return mapId.trim();
    }
    return "default";
  }

  private static String text(JsonNode n, String field) {
    JsonNode v = n.get(field);
    return v != null && v.isTextual() ? v.asText() : null;
  }

  private static Long longVal(JsonNode n, String field) {
    JsonNode v = n.get(field);
    if (v == null || v.isNull()) {
      return 0L;
    }
    return v.asLong();
  }

  private static Double doubleVal(JsonNode n, String field) {
    JsonNode v = n.get(field);
    if (v == null || v.isNull()) {
      return null;
    }
    return v.asDouble();
  }

  private double[] nodeXY(JsonNode node) {
    JsonNode np = node.get("nodePosition");
    if (np != null && np.isObject()) {
      Double px = doubleVal(np, "x");
      Double py = doubleVal(np, "y");
      if (px != null && py != null) {
        Double th = doubleVal(np, "theta");
        if (th != null) {
          theta = th;
        }
        return new double[] {px, py};
      }
    }
    return new double[] {0.0, 0.0};
  }

  public record HeaderIds(long headerId) {}

  private record InstantActionPending(String actionId, String actionType) {}

  /**
   * Follows one or more {@link MotionSegment}s. While moving on a segment, {@code lastNodeId} is
   * the segment start (OpenTCS source point); when the full route completes, {@code lastNodeId} is
   * the final destination point name.
   */
  private static final class RouteFollower {
    final List<MotionSegment> segments;
    final List<String> segFrom;
    final List<String> segTo;
    final List<Double> speedsMps;
    final long firstNodeSeq;
    final long lastNodeSeq;

    double x;
    double y;
    double theta;
    String lastNodeId;
    long lastNodeSequenceId;

    int segIdx;
    double posInSegM;

    /** Straight-line single leg (fallback). */
    static RouteFollower singleStraightLeg(
        String fromId,
        String toId,
        long fromSeq,
        long toSeq,
        double x0,
        double y0,
        double x1,
        double y1,
        double speedMps
    ) {
      PointMm p0 = new PointMm(Math.round(x0 * 1000), Math.round(y0 * 1000));
      PointMm p1 = new PointMm(Math.round(x1 * 1000), Math.round(y1 * 1000));
      PolylineMotion line = new PolylineMotion(java.util.List.of(p0, p1));
      return new RouteFollower(
          java.util.List.of(line),
          java.util.List.of(fromId),
          java.util.List.of(toId),
          java.util.List.of(speedMps),
          fromSeq,
          toSeq
      );
    }

    RouteFollower(
        List<MotionSegment> segments,
        List<String> segFrom,
        List<String> segTo,
        List<Double> speedsMps,
        long firstNodeSeq,
        long lastNodeSeq
    ) {
      if (segments.isEmpty()) {
        throw new IllegalArgumentException("empty route");
      }
      this.segments = segments;
      this.segFrom = segFrom;
      this.segTo = segTo;
      this.speedsMps = speedsMps;
      this.firstNodeSeq = firstNodeSeq;
      this.lastNodeSeq = lastNodeSeq;
    }

    void start() {
      segIdx = 0;
      posInSegM = 0;
      lastNodeSequenceId = firstNodeSeq;
      refreshPose();
    }

    /** @return true when entire route finished */
    boolean advance(double dtSeconds) {
      double remaining = dtSeconds;
      while (remaining > 1e-12 && segIdx < segments.size()) {
        MotionSegment seg = segments.get(segIdx);
        double speed = speedsMps.get(Math.min(segIdx, speedsMps.size() - 1));
        if (speed <= 1e-12) {
          return false;
        }
        double len = Math.max(1e-9, seg.lengthMeters());
        double distLeft = len - posInSegM;
        double timeToExit = distLeft / speed;
        if (timeToExit > remaining) {
          posInSegM += speed * remaining;
          refreshPose();
          lastNodeSequenceId = firstNodeSeq;
          return false;
        }
        remaining -= timeToExit;
        double[] end = seg.sample(len);
        x = end[0];
        y = end[1];
        theta = end[2];
        lastNodeId = segTo.get(segIdx);
        segIdx++;
        posInSegM = 0;
        if (segIdx >= segments.size()) {
          lastNodeSequenceId = lastNodeSeq;
          return true;
        }
      }
      if (segIdx >= segments.size()) {
        lastNodeSequenceId = lastNodeSeq;
        return true;
      }
      refreshPose();
      lastNodeSequenceId = firstNodeSeq;
      return false;
    }

    void refreshPose() {
      MotionSegment seg = segments.get(segIdx);
      double len = seg.lengthMeters();
      double[] p = seg.sample(Math.min(posInSegM, len));
      x = p[0];
      y = p[1];
      theta = p[2];
      lastNodeId = segFrom.get(segIdx);
    }
  }
}
