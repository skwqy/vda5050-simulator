package com.yeefung.vda5050.simulator.map;

/** Point pose in plant coordinates, millimetres (OpenTCS kernel convention). */
public record PointMm(long xMm, long yMm) {

  public double xMeters() {
    return xMm / 1000.0;
  }

  public double yMeters() {
    return yMm / 1000.0;
  }
}
