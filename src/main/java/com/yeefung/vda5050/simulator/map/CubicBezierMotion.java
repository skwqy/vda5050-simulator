package com.yeefung.vda5050.simulator.map;

/**
 * Cubic Bézier P0→P1→P2→P3 in metres. Parameter {@code t} follows distance/length like
 * {@code AgvCommunicationAdapter} (linear mapping of travelled distance to t ∈ [0,1]).
 */
public final class CubicBezierMotion implements MotionSegment {

  private final double x0;
  private final double y0;
  private final double x1;
  private final double y1;
  private final double x2;
  private final double y2;
  private final double x3;
  private final double y3;
  private final double lengthM;

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
    this.lengthM = lengthMmFromXml > 0 ? lengthMmFromXml / 1000.0 : approximateBezierLength();
  }

  private double approximateBezierLength() {
    double len = 0;
    double px = x0;
    double py = y0;
    for (int i = 1; i <= 32; i++) {
      double t = i / 32.0;
      double x = b(t, x0, x1, x2, x3);
      double y = b(t, y0, y1, y2, y3);
      len += Math.hypot(x - px, y - py);
      px = x;
      py = y;
    }
    return len;
  }

  private static double b(double t, double c0, double c1, double c2, double c3) {
    double u = 1 - t;
    return u * u * u * c0 + 3 * u * u * t * c1 + 3 * u * t * t * c2 + t * t * t * c3;
  }

  private static double deriv(double t, double c0, double c1, double c2, double c3) {
    double u = 1 - t;
    return 3 * u * u * (c1 - c0) + 6 * u * t * (c2 - c1) + 3 * t * t * (c3 - c2);
  }

  @Override
  public double lengthMeters() {
    return lengthM;
  }

  @Override
  public double[] sample(double distanceFromStartM) {
    double t = lengthM <= 1e-9 ? 1.0 : Math.max(0, Math.min(1, distanceFromStartM / lengthM));
    double x = b(t, x0, x1, x2, x3);
    double y = b(t, y0, y1, y2, y3);
    double dx = deriv(t, x0, x1, x2, x3);
    double dy = deriv(t, y0, y1, y2, y3);
    double th = Math.atan2(dy, dx);
    return new double[] {x, y, th};
  }
}
