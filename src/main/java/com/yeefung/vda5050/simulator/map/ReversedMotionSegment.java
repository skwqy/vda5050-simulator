package com.yeefung.vda5050.simulator.map;

/**
 * Travels along the same geometry as {@code inner} but from its end toward its start
 * (distance 0 = former {@code inner} end, distance L = former {@code inner} start).
 * Heading is flipped by π so the pose matches travel direction.
 */
public final class ReversedMotionSegment implements MotionSegment {

  private final MotionSegment inner;

  public ReversedMotionSegment(MotionSegment inner) {
    this.inner = inner;
  }

  @Override
  public double lengthMeters() {
    return inner.lengthMeters();
  }

  @Override
  public double[] sample(double distanceFromStartM) {
    double len = inner.lengthMeters();
    double d = Math.max(0, Math.min(distanceFromStartM, len));
    double[] p = inner.sample(len - d);
    p[2] = normalizeAngle(p[2] + Math.PI);
    return p;
  }

  private static double normalizeAngle(double rad) {
    double t = rad % (2 * Math.PI);
    if (t > Math.PI) {
      t -= 2 * Math.PI;
    } else if (t < -Math.PI) {
      t += 2 * Math.PI;
    }
    return t;
  }
}
