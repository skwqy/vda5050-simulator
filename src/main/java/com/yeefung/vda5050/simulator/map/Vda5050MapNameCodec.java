package com.yeefung.vda5050.simulator.map;

/**
 * Mirrors YFAOS {@code com.yeefung.aos...Vda5050MapNameCodec}: OpenTCS plant names vs VDA
 * {@code nodeId}/{@code edgeId} using the same prefix strings and {@code applyStripping} flag.
 */
public final class Vda5050MapNameCodec {

  private final boolean applyStripping;
  private final String pointPrefix;
  private final String pathPrefix;

  public Vda5050MapNameCodec(boolean applyStripping, String pointPrefix, String pathPrefix) {
    this.applyStripping = applyStripping;
    this.pointPrefix = pointPrefix != null ? pointPrefix : "";
    this.pathPrefix = pathPrefix != null ? pathPrefix : "";
  }

  public boolean isApplyStripping() {
    return applyStripping;
  }

  /** OpenTCS point name → VDA {@code nodeId} (outgoing order from AOS; outgoing state from simulator). */
  public String toVehicleNodeId(String openTcsPointName) {
    if (openTcsPointName == null || openTcsPointName.isBlank()) {
      return openTcsPointName;
    }
    if (!applyStripping) {
      return openTcsPointName.trim();
    }
    return stripPrefix(openTcsPointName.trim(), pointPrefix);
  }

  /** OpenTCS path name → VDA {@code edgeId}. */
  public String toVehicleEdgeId(String openTcsPathName) {
    if (openTcsPathName == null || openTcsPathName.isBlank()) {
      return openTcsPathName;
    }
    if (!applyStripping) {
      return openTcsPathName.trim();
    }
    return stripPrefix(openTcsPathName.trim(), pathPrefix);
  }

  /** VDA {@code nodeId} from vehicle → OpenTCS point name (kernel / plant XML keys). */
  public String toOpenTcsPointName(String vehicleReportedNodeId) {
    if (vehicleReportedNodeId == null || vehicleReportedNodeId.isBlank()) {
      return vehicleReportedNodeId;
    }
    if (!applyStripping) {
      return vehicleReportedNodeId.trim();
    }
    String s = vehicleReportedNodeId.trim();
    if (!pointPrefix.isEmpty() && s.startsWith(pointPrefix)) {
      return s;
    }
    return pointPrefix + s;
  }

  /** VDA {@code edgeId} → OpenTCS path name. */
  public String toOpenTcsPathName(String vehicleReportedEdgeId) {
    if (vehicleReportedEdgeId == null || vehicleReportedEdgeId.isBlank()) {
      return vehicleReportedEdgeId;
    }
    if (!applyStripping) {
      return vehicleReportedEdgeId.trim();
    }
    String s = vehicleReportedEdgeId.trim();
    if (!pathPrefix.isEmpty() && s.startsWith(pathPrefix)) {
      return s;
    }
    return pathPrefix + s;
  }

  private static String stripPrefix(String value, String prefix) {
    if (prefix != null && !prefix.isEmpty() && value.startsWith(prefix)) {
      return value.substring(prefix.length());
    }
    return value;
  }
}
