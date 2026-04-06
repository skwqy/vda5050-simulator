package com.yeefung.vda5050.simulator.map;

/**
 * Cubic Bézier P0→P1→P2→P3 in metres. {@link #sample(double)} maps <b>travelled arc length</b> along the
 * curve to a pose. The previous implementation used {@code t = distance / length}, which is not arc
 * length and produced non‑monotonic motion (visible “retreat” toward intermediate nodes).
 */
public final class CubicBezierMotion implements MotionSegment {

  private static final int ARC_INTEGRATION_STEPS = 256;

  private final double x0;
  private final double y0;
  private final double x1;
  private final double y1;
  private final double x2;
  private final double y2;
  private final double x3;
  private final double y3;
  /** Declared path length (often from XML, mm→m); used for {@link #lengthMeters()} and time-to-exit. */
  private final double lengthM;
  /** Numeric arc length P(0)→P(1), consistent with {@link #arcLen0ToT(double)}. */
  private final double arcTotal;

  public CubicBezierMotion(
      PointMm p0,
      PointMm p1,
      PointMm p2,
      PointMm p3,
      long lengthMmFromXml
  ) {
    this.x0 = p0.xMeters();
    this.y0 = p0.yMeters();
    this.x1 = p1.xMeters();
    this.y1 = p1.yMeters();
    this.x2 = p2.xMeters();
    this.y2 = p2.yMeters();
    this.x3 = p3.xMeters();
    this.y3 = p3.yMeters();
    this.arcTotal = arcLen0ToT(1.0);
    double fromXml = lengthMmFromXml > 0 ? lengthMmFromXml / 1000.0 : 0;
    this.lengthM = fromXml > 1e-9 ? fromXml : arcTotal;
  }

  private static double b(double t, double c0, double c1, double c2, double c3) {
    double u = 1 - t;
    return u * u * u * c0 + 3 * u * u * t * c1 + 3 * u * t * t * c2 + t * t * t * c3;
  }

  private static double deriv(double t, double c0, double c1, double c2, double c3) {
    double u = 1 - t;
    return 3 * u * u * (c1 - c0) + 6 * u * t * (c2 - c1) + 3 * t * t * (c3 - c2);
  }

  /**
   * Arc length along the Bézier from parameter 0 to {@code t} (same discretization for all t).
   */
  private double arcLen0ToT(double t) {
    t = Math.max(0, Math.min(1, t));
    double len = 0;
    double px = x0;
    double py = y0;
    for (int j = 1; j <= ARC_INTEGRATION_STEPS; j++) {
      double s = j * t / ARC_INTEGRATION_STEPS;
      double x = b(s, x0, x1, x2, x3);
      double y = b(s, y0, y1, y2, y3);
      len += Math.hypot(x - px, y - py);
      px = x;
      py = y;
    }
    return len;
  }

  /** Find t ∈ [0,1] with arc length from 0 to t equal to {@code targetArc} (metres). */
  private double tForArcLength(double targetArc) {
    if (arcTotal <= 1e-12) {
      return 0;
    }
    if (targetArc <= 0) {
      return 0;
    }
    if (targetArc >= arcTotal) {
      return 1;
    }
    double lo = 0;
    double hi = 1;
    for (int i = 0; i < 40; i++) {
      double mid = 0.5 * (lo + hi);
      if (arcLen0ToT(mid) < targetArc) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    return 0.5 * (lo + hi);
  }

  @Override
  public double lengthMeters() {
    return lengthM;
  }

  @Override
  public double[] sample(double distanceFromStartM) {
    double len = lengthM;
    double d = Math.max(0, Math.min(distanceFromStartM, len));
    double targetArc = arcTotal <= 1e-12 ? 0 : d * (arcTotal / len);
    double t = tForArcLength(targetArc);
    double x = b(t, x0, x1, x2, x3);
    double y = b(t, y0, y1, y2, y3);
    double dx = deriv(t, x0, x1, x2, x3);
    double dy = deriv(t, y0, y1, y2, y3);
    double th = Math.atan2(dy, dx);
    return new double[] {x, y, th};
  }
}
