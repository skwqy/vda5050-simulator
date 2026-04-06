package com.yeefung.vda5050.simulator.map;

import com.yeefung.vda5050.simulator.config.SimulatorProperties;

/**
 * Maps pathLayout control coordinates to world millimetres — same formula as
 * {@code AgvCommunicationAdapter} ({@code x*50}, {@code y*(-50)} by default).
 */
public final class LayoutTransform {

  private final double scaleMm;
  private final boolean flipY;

  public LayoutTransform(SimulatorProperties.Map map) {
    this.scaleMm = map.layoutScaleMm > 0 ? map.layoutScaleMm : 50.0;
    this.flipY = map.layoutFlipY;
  }

  public long toWorldXMm(double layoutX) {
    return Math.round(layoutX * scaleMm);
  }

  public long toWorldYMm(double layoutY) {
    double w = flipY ? (-layoutY * scaleMm) : (layoutY * scaleMm);
    return Math.round(w);
  }
}
