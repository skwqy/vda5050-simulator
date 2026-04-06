package com.yeefung.vda5050.simulator.map;

import com.yeefung.vda5050.simulator.map.PlantPath.ConnectionType;
import com.yeefung.vda5050.simulator.map.PlantPath.LayoutControlPoint;
import java.util.ArrayList;
import java.util.List;

/** Builds {@link MotionSegment} from plant geometry (aligned with YFAOS {@code AgvCommunicationAdapter}). */
public final class PathMotionFactory {

  private PathMotionFactory() {}

  public static MotionSegment build(PlantPath path, PlantModel model, LayoutTransform tf) {
    PointMm src =
        model.point(path.sourcePointName()).orElseThrow(() -> new IllegalArgumentException(
            "Unknown point: " + path.sourcePointName()));
    PointMm dst =
        model.point(path.destinationPointName()).orElseThrow(() -> new IllegalArgumentException(
            "Unknown point: " + path.destinationPointName()));

    return switch (path.connectionType()) {
      case DIRECT -> polylineDirect(src, dst);
      case POLYPATH -> {
        List<PointMm> world = new ArrayList<>();
        world.add(src);
        for (LayoutControlPoint c : path.layoutControlPoints()) {
          world.add(new PointMm(tf.toWorldXMm(c.layoutX()), tf.toWorldYMm(c.layoutY())));
        }
        world.add(dst);
        yield new PolylineMotion(world);
      }
      case BEZIER -> {
        List<LayoutControlPoint> cps = path.layoutControlPoints();
        if (cps.size() < 2) {
          yield polylineDirect(src, dst);
        }
        PointMm p1 = new PointMm(tf.toWorldXMm(cps.get(0).layoutX()), tf.toWorldYMm(cps.get(0).layoutY()));
        PointMm p2 = new PointMm(tf.toWorldXMm(cps.get(1).layoutX()), tf.toWorldYMm(cps.get(1).layoutY()));
        yield new CubicBezierMotion(src, p1, p2, dst, path.lengthMm());
      }
    };
  }

  private static PolylineMotion polylineDirect(PointMm src, PointMm dst) {
    List<PointMm> w = new ArrayList<>(2);
    w.add(src);
    w.add(dst);
    return new PolylineMotion(w);
  }
}
