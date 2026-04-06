package com.yeefung.vda5050.simulator.map;

import java.util.List;

/**
 * One logical path between two points, with layout control points in <strong>layout</strong>
 * coordinates (before {@link LayoutTransform}).
 */
public record PlantPath(
    String name,
    String sourcePointName,
    String destinationPointName,
    long lengthMm,
    ConnectionType connectionType,
    List<LayoutControlPoint> layoutControlPoints
) {
  public enum ConnectionType {
    POLYPATH,
    BEZIER,
    DIRECT
  }

  public record LayoutControlPoint(double layoutX, double layoutY) {}
}
