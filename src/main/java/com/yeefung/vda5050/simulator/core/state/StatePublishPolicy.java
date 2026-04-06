package com.yeefung.vda5050.simulator.core.state;

import com.yeefung.vda5050.simulator.map.Vda5050MapNameCodec;
import java.util.Objects;

/**
 * Decides when to emit {@code state} (event-driven, not periodic). Continuation-order standstill
 * suppression is explicit — not a large distance heuristic.
 *
 * <p>Evaluation order: pending {@code actionStates} (e.g. AOS {@code instantActions} replies) and
 * {@link #setImmediateStatePublishRequested(boolean)} run <b>before</b> IGNORE_RELEASED intermediate
 * waypoint suppression — timely command acknowledgements still produce MQTT {@code state}.
 */
public final class StatePublishPolicy {

  private boolean statePublishedOnce;
  private String lastPublishedOpenTcsNodeId = "";
  private long lastPublishedNodeSequenceId;
  private double lastPublishedX = Double.NaN;
  private double lastPublishedY = Double.NaN;
  private String lastPublishedOrderId = "";
  private boolean suppressOneStatePublishAfterStandstillContinuation;
  private boolean requestImmediateStatePublish;

  /**
   * @param x current pose m (world)
   * @param y current pose m (world)
   * @param currentOrderId engine {@code orderId} after the latest MQTT order
   * @param ignoreReleasedSuppressIntermediateNode when {@code true} (IGNORE_RELEASED + stopped at a
   *     non-final waypoint), sync dedupe snapshot but do not emit MQTT {@code state} for this node
   *     alone — does not apply when {@code hasPendingActionStatesToEmbed} or immediate publish is set
   */
  public boolean shouldPublish(
      boolean hasPendingActionStatesToEmbed,
      Vda5050MapNameCodec mapNameCodec,
      String currentOpenTcsLastNodeId,
      long currentLastNodeSequenceId,
      double x,
      double y,
      String currentOrderId,
      boolean ignoreReleasedSuppressIntermediateNode
  ) {
    if (hasPendingActionStatesToEmbed) {
      suppressOneStatePublishAfterStandstillContinuation = false;
      return true;
    }
    if (requestImmediateStatePublish) {
      suppressOneStatePublishAfterStandstillContinuation = false;
      return true;
    }
    if (!statePublishedOnce) {
      return true;
    }
    if (suppressOneStatePublishAfterStandstillContinuation) {
      suppressOneStatePublishAfterStandstillContinuation = false;
      // Skip MQTT once, but advance dedupe snapshot — otherwise the next simulation tick sees
      // lastNodeSequenceId / orderId drift vs lastPublished* and emits another state at the same pose.
      recordSuccessfulPublish(currentOpenTcsLastNodeId, currentLastNodeSequenceId, x, y, currentOrderId);
      return false;
    }
    if (ignoreReleasedSuppressIntermediateNode) {
      recordSuccessfulPublish(currentOpenTcsLastNodeId, currentLastNodeSequenceId, x, y, currentOrderId);
      return false;
    }
    boolean nodeChanged = !sameLogicalOpenTcsNodeId(currentOpenTcsLastNodeId, lastPublishedOpenTcsNodeId, mapNameCodec);
    if (!nodeChanged && currentLastNodeSequenceId == lastPublishedNodeSequenceId) {
      return false;
    }
    return true;
  }

  public void recordSuccessfulPublish(
      String openTcsLastNodeId,
      long lastNodeSequenceId,
      double x,
      double y,
      String orderId
  ) {
    requestImmediateStatePublish = false;
    statePublishedOnce = true;
    lastPublishedOpenTcsNodeId = openTcsLastNodeId;
    lastPublishedNodeSequenceId = lastNodeSequenceId;
    lastPublishedX = x;
    lastPublishedY = y;
    lastPublishedOrderId = orderId != null ? orderId : "";
  }

  /**
   * Call after a new MQTT {@code order} was applied and the route was restarted at standstill on the
   * same logical node — may set {@link #shouldPublish} to skip the next publish once.
   */
  public void evaluateStandstillContinuationAfterOrder(
      String newOrderId,
      String currentOpenTcsLastNodeId,
      double x,
      double y,
      Vda5050MapNameCodec codec,
      double continuationStandstillEpsilonM
  ) {
    if (!statePublishedOnce || lastPublishedOrderId.isEmpty()) {
      return;
    }
    if (newOrderId == null || newOrderId.equals(lastPublishedOrderId)) {
      return;
    }
    if (!sameLogicalOpenTcsNodeId(currentOpenTcsLastNodeId, lastPublishedOpenTcsNodeId, codec)) {
      return;
    }
    if (!poseWithinEpsilonM(x, y, lastPublishedX, lastPublishedY, continuationStandstillEpsilonM)) {
      return;
    }
    suppressOneStatePublishAfterStandstillContinuation = true;
  }

  public void setImmediateStatePublishRequested(boolean value) {
    requestImmediateStatePublish = value;
  }

  private static boolean sameLogicalOpenTcsNodeId(String a, String b, Vda5050MapNameCodec mapNameCodec) {
    if (Objects.equals(a, b)) {
      return true;
    }
    if (a == null || b == null || a.isBlank() || b.isBlank()) {
      return false;
    }
    return mapNameCodec.toVehicleNodeId(a.trim()).equals(mapNameCodec.toVehicleNodeId(b.trim()));
  }

  private static boolean poseWithinEpsilonM(
      double x, double y, double lastX, double lastY, double epsilonM
  ) {
    if (Double.isNaN(lastX) || Double.isNaN(lastY)) {
      return false;
    }
    double dx = x - lastX;
    double dy = y - lastY;
    return dx * dx + dy * dy <= epsilonM * epsilonM;
  }
}
