package com.yeefung.vda5050.simulator.core;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.JsonNodeFactory;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.yeefung.vda5050.simulator.config.OrderExecutionMode;
import com.yeefung.vda5050.simulator.config.SimulatorProperties;
import com.yeefung.vda5050.simulator.core.order.IgnoreReleasedActiveEdgeTrimmer;
import com.yeefung.vda5050.simulator.core.order.OrderRoutePlanner;
import com.yeefung.vda5050.simulator.core.order.ReleasedEdgeSelector;
import com.yeefung.vda5050.simulator.core.route.RouteFollower;
import com.yeefung.vda5050.simulator.core.state.StatePublishPolicy;
import com.yeefung.vda5050.simulator.core.vda.VdaHeaderIds;
import com.yeefung.vda5050.simulator.core.vda.VdaMessageBuilder;
import com.yeefung.vda5050.simulator.map.PlantModel;
import com.yeefung.vda5050.simulator.map.PointMm;
import com.yeefung.vda5050.simulator.map.Vda5050MapNameCodec;
import com.yeefung.vda5050.simulator.util.JsonNodes;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Queue;

/**
 * Movement compatible with YFAOS {@code MessageResponseMatcher#orderAccepted}: {@code lastNodeId}
 * must reach the segment destination. When a plant XML is loaded, motion follows POLYPATH / BEZIER
 * geometry like {@code AgvCommunicationAdapter}.
 *
 * <p>State publication cadence is delegated to {@link com.yeefung.vda5050.simulator.core.state.StatePublishPolicy};
 * VDA JSON payloads are built by the injected {@link com.yeefung.vda5050.simulator.core.vda.VdaMessageBuilder}.
 * Order edge selection and route geometry use {@link com.yeefung.vda5050.simulator.core.order.ReleasedEdgeSelector}
 * and the injected {@link com.yeefung.vda5050.simulator.core.order.OrderRoutePlanner} (sharing {@link PlantBinding}
 * wiring with the planner).
 */
public final class SimulationEngine {

  private static final System.Logger LOG = System.getLogger(SimulationEngine.class.getName());

  private final SimulatorProperties props;
  private final PlantModel plantModel;
  private final PlantModel.NameResolver nameResolver;
  private final Vda5050MapNameCodec mapNameCodec;
  private final JsonNodeFactory json = JsonNodeFactory.instance;
  private final StatePublishPolicy statePublishPolicy = new StatePublishPolicy();
  private final VdaMessageBuilder vdaMessageBuilder;
  private final ReleasedEdgeSelector releasedEdgeSelector;
  private final OrderRoutePlanner orderRoutePlanner;

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

  public SimulationEngine(
      SimulatorProperties props,
      PlantModel plantModel,
      VdaMessageBuilder vdaMessageBuilder,
      OrderRoutePlanner orderRoutePlanner,
      PlantBinding plantBinding
  ) {
    this.props = Objects.requireNonNull(props, "props");
    this.plantModel = plantModel;
    Objects.requireNonNull(plantBinding, "plantBinding");
    this.mapNameCodec = plantBinding.mapNameCodec();
    this.nameResolver = plantBinding.nameResolver();
    this.vdaMessageBuilder = Objects.requireNonNull(vdaMessageBuilder, "vdaMessageBuilder");
    this.orderRoutePlanner = Objects.requireNonNull(orderRoutePlanner, "orderRoutePlanner");
    this.releasedEdgeSelector = new ReleasedEdgeSelector(props);
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
   * point ({@code lastNodeId} / {@code lastNodeSequenceId} change), except in
   * {@link OrderExecutionMode#IGNORE_RELEASED} at intermediate waypoints (multi-edge routes).
   * Pending {@code instantActions} / {@code actionStates} and other high-priority branches in
   * {@link StatePublishPolicy} still publish first. Delegates to {@link StatePublishPolicy}.
   */
  public synchronized boolean shouldPublishState() {
    flushInstantToActionStates();
    boolean ignoreReleasedIntermediate =
        props.simulation.orderExecutionMode == OrderExecutionMode.IGNORE_RELEASED
            && route != null
            && route.isAtIntermediateWaypoint();
    return statePublishPolicy.shouldPublish(
        !lastActionStates.isEmpty(),
        mapNameCodec,
        lastNodeId,
        lastNodeSequenceId,
        x,
        y,
        orderId,
        ignoreReleasedIntermediate);
  }

  /** Call after a {@code state} payload was successfully sent. */
  public synchronized void afterStatePublished() {
    statePublishPolicy.recordSuccessfulPublish(lastNodeId, lastNodeSequenceId, x, y, orderId);
  }

  public synchronized void onOrderMessage(JsonNode root) {
    String oid = JsonNodes.text(root, "orderId");
    long ouid = JsonNodes.longVal(root, "orderUpdateId");
    List<JsonNode> sortedNodes = ReleasedEdgeSelector.sortedOrderNodes(root);
    if (sortedNodes.isEmpty()) {
      return;
    }

    orderId = oid != null ? oid : "";
    orderUpdateId = ouid;

    List<JsonNode> sortedEdges = releasedEdgeSelector.sortEdgesBySequenceId(root);
    if (sortedEdges.isEmpty()) {
      LOG.log(
          System.Logger.Level.WARNING,
          "order has no edges — skip motion"
      );
      route = null;
      driving = false;
      return;
    }

    List<JsonNode> activeEdges = new ArrayList<>(releasedEdgeSelector.selectEdgesForExecution(sortedEdges));

    if (props.simulation.orderExecutionMode == OrderExecutionMode.IGNORE_RELEASED
        && route != null
        && driving
        && !activeEdges.isEmpty()) {
      int skip =
          IgnoreReleasedActiveEdgeTrimmer.computeLeadingSkipCount(
              activeEdges, sortedNodes, route, orderRoutePlanner, mapNameCodec);
      if (skip >= activeEdges.size()) {
        orderId = oid != null ? oid : "";
        orderUpdateId = ouid;
        LOG.log(
            System.Logger.Level.INFO,
            "IGNORE_RELEASED: new order edges already covered — keep current route");
        return;
      }
      if (skip > 0) {
        activeEdges = new ArrayList<>(activeEdges.subList(skip, activeEdges.size()));
        LOG.log(
            System.Logger.Level.INFO,
            "IGNORE_RELEASED: skipping " + skip + " leading edge(s) already passed");
      }
    }

    if (activeEdges.isEmpty()) {
      if (props.simulation.orderExecutionMode == OrderExecutionMode.STRICT_PREFIX_RELEASED) {
        JsonNode first = sortedEdges.getFirst();
        long brk = JsonNodes.longVal(first, "sequenceId");
        LOG.log(
            System.Logger.Level.WARNING,
            "STRICT_PREFIX_RELEASED: empty released prefix orderId="
                + orderId
                + " orderUpdateId="
                + orderUpdateId
                + " breakpointSequenceId="
                + brk
                + " lastNodeId="
                + lastNodeId
        );
        statePublishPolicy.setImmediateStatePublishRequested(true);
      } else {
        LOG.log(System.Logger.Level.WARNING, "order has no executable edges — skip motion");
      }
      route = null;
      driving = false;
      return;
    }

    if (props.simulation.orderExecutionMode == OrderExecutionMode.STRICT_PREFIX_RELEASED
        && sortedEdges.size() > activeEdges.size()) {
      JsonNode nextAfterPrefix = sortedEdges.get(activeEdges.size());
      LOG.log(
          System.Logger.Level.INFO,
          "STRICT_PREFIX_RELEASED: horizon boundary orderId="
              + orderId
              + " prefixEdges="
              + activeEdges.size()
              + " totalEdges="
              + sortedEdges.size()
              + " nextUnreleasedSequenceId="
              + JsonNodes.longVal(nextAfterPrefix, "sequenceId")
      );
    }

    double speed = OrderRoutePlanner.maxEdgeSpeedMps(activeEdges, props.simulation.defaultSpeedMps);

    JsonNode firstReleasedStartNode =
        orderRoutePlanner.findOrderNode(
            sortedNodes, JsonNodes.text(activeEdges.getFirst(), "startNodeId"));
    JsonNode lastReleasedEndNode =
        orderRoutePlanner.findOrderNode(
            sortedNodes, JsonNodes.text(activeEdges.getLast(), "endNodeId"));
    if (firstReleasedStartNode == null || lastReleasedEndNode == null) {
      System.getLogger(SimulationEngine.class.getName()).log(
          System.Logger.Level.WARNING,
          "order nodes missing for released edge endpoints");
      route = null;
      driving = false;
      return;
    }

    Optional<RouteFollower> built =
        orderRoutePlanner.buildRoute(
            activeEdges, sortedNodes, firstReleasedStartNode, lastReleasedEndNode, speed);
    if (built.isEmpty()) {
      route = null;
      driving = false;
      return;
    }

    route = built.get();
    double wx = x;
    double wy = y;
    route.start();
    if (props.simulation.orderExecutionMode == OrderExecutionMode.IGNORE_RELEASED && driving) {
      route.snapProgressToWorld(wx, wy);
    }
    x = route.x;
    y = route.y;
    theta = route.theta;
    lastNodeId = route.lastNodeId;
    // Must match RouteFollower (segFromSeq); do not use nodes[] alone — it can disagree and then
    // the next tick() sync would flip lastNodeSequenceId and trigger a duplicate state at the same pose.
    lastNodeSequenceId = route.lastNodeSequenceId;
    driving = true;

    statePublishPolicy.evaluateStandstillContinuationAfterOrder(
        orderId,
        lastNodeId,
        x,
        y,
        mapNameCodec,
        props.simulation.continuationStandstillEpsilonM);
  }


  public synchronized void onInstantActionsMessage(JsonNode root) {
    JsonNode actions = root.get("actions");
    if (actions == null || !actions.isArray()) {
      return;
    }
    for (JsonNode a : actions) {
      String type = JsonNodes.text(a, "actionType");
      String aid = JsonNodes.text(a, "actionId");
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

  public synchronized ObjectNode buildState(VdaHeaderIds ids) {
    flushInstantToActionStates();
    ArrayNode actionStates = json.arrayNode();
    for (ObjectNode a : lastActionStates) {
      actionStates.add(a);
    }
    lastActionStates.clear();

    return vdaMessageBuilder.buildState(
        props,
        mapNameCodec,
        ids,
        orderId,
        orderUpdateId,
        lastNodeId,
        lastNodeSequenceId,
        driving,
        actionStates,
        x,
        y,
        theta,
        effectiveMapIdForJson());
  }

  public synchronized ObjectNode buildVisualization(VdaHeaderIds ids) {
    return vdaMessageBuilder.buildVisualization(
        props,
        ids,
        x,
        y,
        theta,
        effectiveMapIdForJson());
  }

  /** VDA / AOS schema: {@code agvPosition.mapId} is required; never omit or send blank. */
  private String effectiveMapIdForJson() {
    if (mapId != null && !mapId.isBlank()) {
      return mapId.trim();
    }
    return "default";
  }

  private record InstantActionPending(String actionId, String actionType) {}

}
