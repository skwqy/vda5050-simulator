package com.yeefung.vda5050.simulator.map;

import java.util.ArrayList;
import java.util.List;

/** Polyline in world metres: 起点 → 控制点 → 终点（与 POLYPATH 一致）. */
public final class PolylineMotion implements MotionSegment {

  private final double[] px;
  private final double[] py;
  private final double[] cum;
  private final double totalLen;

  public PolylineMotion(List<PointMm> worldPointsMm) {
    if (worldPointsMm.size() < 2) {
      throw new IllegalArgumentException("polyline needs >= 2 points");
    }
    int n = worldPointsMm.size();
    px = new double[n];
    py = new double[n];
    for (int i = 0; i < n; i++) {
      px[i] = worldPointsMm.get(i).xMeters();
      py[i] = worldPointsMm.get(i).yMeters();
    }
    cum = new double[n];
    cum[0] = 0;
    double acc = 0;
    for (int i = 1; i < n; i++) {
      double dx = px[i] - px[i - 1];
      double dy = py[i] - py[i - 1];
      acc += Math.hypot(dx, dy);
      cum[i] = acc;
    }
    totalLen = acc;
  }

  @Override
  public double lengthMeters() {
    return totalLen;
  }

  @Override
  public double[] sample(double distanceFromStartM) {
    if (totalLen <= 1e-9) {
      return new double[] {px[0], py[0], theta(px[0], py[0], px[1], py[1])};
    }
    double d = Math.max(0, Math.min(distanceFromStartM, totalLen));
    if (d >= totalLen - 1e-9) {
      int last = px.length - 1;
      return new double[] {
          px[last], py[last], theta(px[last - 1], py[last - 1], px[last], py[last])
      };
    }
    for (int i = 1; i < cum.length; i++) {
      if (d <= cum[i] + 1e-9) {
        double segStart = cum[i - 1];
        double segLen = cum[i] - segStart;
        double t = segLen > 1e-12 ? (d - segStart) / segLen : 0;
        double x = px[i - 1] + t * (px[i] - px[i - 1]);
        double y = py[i - 1] + t * (py[i] - py[i - 1]);
        double th = theta(px[i - 1], py[i - 1], px[i], py[i]);
        return new double[] {x, y, th};
      }
    }
    int last = px.length - 1;
    return new double[] {px[last], py[last], theta(px[last - 1], py[last - 1], px[last], py[last])};
  }

  private static double theta(double x0, double y0, double x1, double y1) {
    return Math.atan2(y1 - y0, x1 - x0);
  }

  public static PolylineMotion fromWorldMm(List<PointMm> mm) {
    return new PolylineMotion(new ArrayList<>(mm));
  }
}
