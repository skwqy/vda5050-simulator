package com.yeefung.vda5050.simulator.core.vda;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.JsonNodeFactory;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.yeefung.vda5050.simulator.config.SimulatorProperties;
import com.yeefung.vda5050.simulator.map.Vda5050MapNameCodec;

/**
 * Builds VDA 2.1 JSON payloads for outgoing topics ({@code connection}, {@code state},
 * {@code visualization}); no simulation logic.
 */
public final class VdaMessageBuilder {

  private final JsonNodeFactory json = JsonNodeFactory.instance;

  public ObjectNode buildConnection(SimulatorProperties props, VdaHeaderIds ids) {
    ObjectNode root = json.objectNode();
    putCommonVehicleHeader(root, props, ids);
    root.put("connectionState", "ONLINE");
    return root;
  }

  public ObjectNode buildState(
      SimulatorProperties props,
      Vda5050MapNameCodec mapNameCodec,
      VdaHeaderIds ids,
      String orderId,
      long orderUpdateId,
      String lastNodeIdOpenTcs,
      long lastNodeSequenceId,
      boolean driving,
      ArrayNode actionStates,
      double x,
      double y,
      double theta,
      String mapIdForJson
  ) {
    ObjectNode root = json.objectNode();
    putCommonVehicleHeader(root, props, ids);

    root.put("orderId", orderId);
    root.put("orderUpdateId", orderUpdateId);
    root.put(
        "lastNodeId",
        lastNodeIdOpenTcs.isEmpty() ? "" : mapNameCodec.toVehicleNodeId(lastNodeIdOpenTcs));
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
    pos.put("mapId", mapIdForJson);
    pos.put("positionInitialized", true);
    root.set("agvPosition", pos);

    if (orderId != null && !orderId.isEmpty()) {
      root.put("orderState", driving ? "ACTIVE" : "ACTIVE");
    }

    return root;
  }

  public ObjectNode buildVisualization(
      SimulatorProperties props,
      VdaHeaderIds ids,
      double x,
      double y,
      double theta,
      String mapIdForJson
  ) {
    ObjectNode root = json.objectNode();
    putCommonVehicleHeader(root, props, ids);
    ObjectNode pos = json.objectNode();
    pos.put("x", x);
    pos.put("y", y);
    pos.put("theta", theta);
    pos.put("mapId", mapIdForJson);
    pos.put("positionInitialized", true);
    root.set("agvPosition", pos);
    return root;
  }

  private static void putCommonVehicleHeader(
      ObjectNode root, SimulatorProperties props, VdaHeaderIds ids
  ) {
    root.put("headerId", ids.headerId());
    root.put("timestamp", java.time.Instant.now().toString());
    root.put("version", props.simulation.protocolVersion);
    root.put("manufacturer", props.mqtt.manufacturer);
    root.put("serialNumber", props.mqtt.serialNumber);
  }
}
