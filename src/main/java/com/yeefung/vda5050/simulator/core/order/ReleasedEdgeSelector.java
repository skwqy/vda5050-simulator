package com.yeefung.vda5050.simulator.core.order;

import com.fasterxml.jackson.databind.JsonNode;
import com.yeefung.vda5050.simulator.config.OrderExecutionMode;
import com.yeefung.vda5050.simulator.config.SimulatorProperties;
import com.yeefung.vda5050.simulator.util.JsonNodes;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/** Selects which order edges are executable under {@link OrderExecutionMode} and {@code released}. */
public final class ReleasedEdgeSelector {

  private final SimulatorProperties props;

  public ReleasedEdgeSelector(SimulatorProperties props) {
    this.props = props;
  }

  /** Returns empty list if {@code nodes} is missing, not an array, or empty. */
  public static List<JsonNode> sortedOrderNodes(JsonNode orderRoot) {
    JsonNode nodes = orderRoot.get("nodes");
    if (nodes == null || !nodes.isArray() || nodes.isEmpty()) {
      return List.of();
    }
    List<JsonNode> sortedNodes = new ArrayList<>();
    nodes.forEach(sortedNodes::add);
    sortedNodes.sort(Comparator.comparingLong(n -> JsonNodes.longVal(n, "sequenceId")));
    return sortedNodes;
  }

  public List<JsonNode> sortEdgesBySequenceId(JsonNode orderRoot) {
    JsonNode edgesNode = orderRoot.get("edges");
    if (edgesNode == null || !edgesNode.isArray()) {
      return List.of();
    }
    List<JsonNode> sorted = new ArrayList<>();
    for (JsonNode e : edgesNode) {
      sorted.add(e);
    }
    sorted.sort(Comparator.comparingLong(e -> JsonNodes.longVal(e, "sequenceId")));
    return sorted;
  }

  public List<JsonNode> selectEdgesForExecution(List<JsonNode> sortedEdges) {
    if (sortedEdges.isEmpty()) {
      return List.of();
    }
    if (props.simulation.orderExecutionMode == OrderExecutionMode.IGNORE_RELEASED) {
      return new ArrayList<>(sortedEdges);
    }
    List<JsonNode> prefix = new ArrayList<>();
    for (JsonNode e : sortedEdges) {
      if (!edgeReleasedStrict(e)) {
        break;
      }
      prefix.add(e);
    }
    return prefix;
  }

  /** Strict horizon: missing {@code released} follows {@link SimulatorProperties.Simulation#releasedFieldMissingAsReleased}. */
  private boolean edgeReleasedStrict(JsonNode edge) {
    JsonNode r = edge.get("released");
    if (r == null || r.isNull()) {
      return props.simulation.releasedFieldMissingAsReleased;
    }
    return r.asBoolean();
  }
}
