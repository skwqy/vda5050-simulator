package com.yeefung.vda5050.simulator.map;

/**
 * Parametric motion along one path segment; distances in metres, theta in radians.
 */
public interface MotionSegment {

  /** Total geometric length used for pacing (metres). */
  double lengthMeters();

  /**
   * @param distanceFromStartM distance along this segment [0, length]
   * @return x, y in metres, theta in radians
   */
  double[] sample(double distanceFromStartM);
}
