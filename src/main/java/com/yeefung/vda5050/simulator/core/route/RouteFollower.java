package com.yeefung.vda5050.simulator.core.route;

import com.yeefung.vda5050.simulator.map.MotionSegment;
import com.yeefung.vda5050.simulator.map.PointMm;
import com.yeefung.vda5050.simulator.map.PolylineMotion;
import java.util.List;

/**
 * Follows one or more {@link MotionSegment}s. While moving on a segment, {@code lastNodeId} is the
 * segment start (OpenTCS source point); when the full route completes, {@code lastNodeId} is the
 * final destination point name.
 *
 * <p>Each edge’s {@link MotionSegment} may have {@code sample(0)} slightly different from the previous
 * edge’s end (plant vs order rounding). A per-segment XY translation aligns {@code sample(0)} to the
 * pose reached at the junction so motion does not snap backward at intermediate nodes.
 */
public final class RouteFollower {
  final List<MotionSegment> segments;
  final List<String> segFrom;
  final List<String> segTo;
  final List<Double> speedsMps;
  /** Order {@code sequenceId} for the start node of each edge (segment). */
  final List<Long> segFromSeq;
  /** Order {@code sequenceId} for the end node of each edge. */
  final List<Long> segToSeq;

  public double x;
  public double y;
  public double theta;
  public String lastNodeId;
  public long lastNodeSequenceId;

  int segIdx;
  double posInSegM;

  /** Start-of-current-segment pose: matches previous segment's end at junctions (avoids Bezier sample drift). */
  private double segEntryX;
  private double segEntryY;
  private double segEntryTheta;

  /**
   * Translates raw {@link MotionSegment#sample(double)} so the segment’s geometric start matches
   * {@link #segEntryX}/{@link #segEntryY} at each junction (after the first edge).
   */
  private double segOffsetX;
  private double segOffsetY;

  /** Straight-line single leg (fallback). */
  public static RouteFollower singleStraightLeg(
      String fromId,
      String toId,
      long fromSeq,
      long toSeq,
      double x0,
      double y0,
      double x1,
      double y1,
      double speedMps
  ) {
    PointMm p0 = new PointMm(Math.round(x0 * 1000), Math.round(y0 * 1000));
    PointMm p1 = new PointMm(Math.round(x1 * 1000), Math.round(y1 * 1000));
    PolylineMotion line = new PolylineMotion(List.of(p0, p1));
    return new RouteFollower(
        List.of(line),
        List.of(fromId),
        List.of(toId),
        List.of(speedMps),
        List.of(fromSeq),
        List.of(toSeq)
    );
  }

  public RouteFollower(
      List<MotionSegment> segments,
      List<String> segFrom,
      List<String> segTo,
      List<Double> speedsMps,
      List<Long> segFromSeq,
      List<Long> segToSeq
  ) {
    if (segments.isEmpty()) {
      throw new IllegalArgumentException("empty route");
    }
    if (segFromSeq.size() != segments.size() || segToSeq.size() != segments.size()) {
      throw new IllegalArgumentException("segment sequence ids size mismatch");
    }
    this.segments = segments;
    this.segFrom = segFrom;
    this.segTo = segTo;
    this.speedsMps = speedsMps;
    this.segFromSeq = segFromSeq;
    this.segToSeq = segToSeq;
  }

  public void start() {
    segIdx = 0;
    posInSegM = 0;
    MotionSegment s0 = segments.getFirst();
    double[] p0 = s0.sample(0);
    segEntryX = p0[0];
    segEntryY = p0[1];
    segEntryTheta = p0[2];
    segOffsetX = 0;
    segOffsetY = 0;
    refreshPose();
  }

  /** Segment index of the edge currently being followed (0-based). */
  public int getCurrentSegmentIndex() {
    return segIdx;
  }

  public int getSegmentCount() {
    return segments.size();
  }

  public String getSegFrom(int i) {
    return segFrom.get(i);
  }

  public String getSegTo(int i) {
    return segTo.get(i);
  }

  /**
   * After {@link #start()}, sets {@code posInSegM} so the pose on the first segment is closest to
   * {@code wx, wy} — used when IGNORE_RELEASED trims leading edges and the vehicle is mid-edge.
   */
  public void snapProgressToWorld(double wx, double wy) {
    if (segments.isEmpty()) {
      return;
    }
    segIdx = 0;
    MotionSegment seg = segments.getFirst();
    double len = Math.max(1e-9, seg.lengthMeters());
    double bestD = 0;
    double bestDist = Double.MAX_VALUE;
    for (int k = 0; k <= 96; k++) {
      double d = len * k / 96.0;
      double[] p = seg.sample(d);
      double px = p[0] + segOffsetX;
      double py = p[1] + segOffsetY;
      double dist = (px - wx) * (px - wx) + (py - wy) * (py - wy);
      if (dist < bestDist) {
        bestDist = dist;
        bestD = d;
      }
    }
    posInSegM = Math.min(bestD, len);
    refreshPose();
  }

  private void recomputeSegOffsetFromEntry() {
    MotionSegment seg = segments.get(segIdx);
    double[] r0 = seg.sample(0);
    segOffsetX = segEntryX - r0[0];
    segOffsetY = segEntryY - r0[1];
  }

  /**
   * After completing an edge but before the route ends: vehicle is at a path-internal node (e.g. point 2
   * on 1→2→3). Used by {@code IGNORE_RELEASED} to skip MQTT {@code state} driven only by node changes at
   * intermediates. Does not block {@code state} that carries {@code actionStates} (e.g. AOS
   * {@code instantActions} replies) — those are handled first in {@link com.yeefung.vda5050.simulator.core.state.StatePublishPolicy}.
   */
  public boolean isAtIntermediateWaypoint() {
    return segments.size() > 1 && segIdx > 0 && segIdx < segments.size();
  }

  /** @return true when entire route finished */
  public boolean advance(double dtSeconds) {
    double remaining = dtSeconds;
    while (remaining > 1e-12 && segIdx < segments.size()) {
      MotionSegment seg = segments.get(segIdx);
      double speed = speedsMps.get(Math.min(segIdx, speedsMps.size() - 1));
      if (speed <= 1e-12) {
        return false;
      }
      double len = Math.max(1e-9, seg.lengthMeters());
      double distLeft = len - posInSegM;
      double timeToExit = distLeft / speed;
      if (timeToExit > remaining) {
        posInSegM += speed * remaining;
        refreshPose();
        return false;
      }
      remaining -= timeToExit;
      double[] end = seg.sample(len);
      x = end[0] + segOffsetX;
      y = end[1] + segOffsetY;
      theta = end[2];
      lastNodeId = segTo.get(segIdx);
      lastNodeSequenceId = segToSeq.get(segIdx);
      segEntryX = x;
      segEntryY = y;
      segEntryTheta = theta;
      segIdx++;
      posInSegM = 0;
      if (segIdx >= segments.size()) {
        return true;
      }
      recomputeSegOffsetFromEntry();
    }
    if (segIdx >= segments.size()) {
      return true;
    }
    refreshPose();
    return false;
  }

  private void refreshPose() {
    MotionSegment seg = segments.get(segIdx);
    double len = Math.max(1e-9, seg.lengthMeters());
    double tdist = Math.min(posInSegM, len);
    if (tdist <= 1e-9) {
      x = segEntryX;
      y = segEntryY;
      theta = segEntryTheta;
      lastNodeId = segFrom.get(segIdx);
      lastNodeSequenceId = segFromSeq.get(segIdx);
      return;
    }
    double[] p = seg.sample(tdist);
    x = p[0] + segOffsetX;
    y = p[1] + segOffsetY;
    theta = p[2];
    lastNodeId = segFrom.get(segIdx);
    lastNodeSequenceId = segFromSeq.get(segIdx);
  }
}
