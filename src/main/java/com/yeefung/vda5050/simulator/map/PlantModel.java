package com.yeefung.vda5050.simulator.map;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/**
 * Subset of an OpenTCS driving course: points (mm) and paths with layout for POLYPATH / BEZIER.
 */
public final class PlantModel {

  private final String modelName;
  private final Map<String, PointMm> pointsByName;
  private final Map<String, PlantPath> pathsByName;

  public PlantModel(
      String modelName,
      Map<String, PointMm> pointsByName,
      Map<String, PlantPath> pathsByName
  ) {
    this.modelName = modelName != null ? modelName : "";
    this.pointsByName = Map.copyOf(pointsByName);
    this.pathsByName = Map.copyOf(pathsByName);
  }

  public String getModelName() {
    return modelName;
  }

  public Optional<PointMm> point(String openTcsName) {
    return Optional.ofNullable(pointsByName.get(openTcsName));
  }

  public Optional<PlantPath> pathByName(String openTcsPathName) {
    return Optional.ofNullable(pathsByName.get(openTcsPathName));
  }

  public int pointCount() {
    return pointsByName.size();
  }

  public int pathCount() {
    return pathsByName.size();
  }

  /**
   * Resolves a path for a VDA edge. Prefers matching {@code startNodeId}/{@code endNodeId} to the
   * plant (avoids picking a wrong path when {@code edgeId} collides with an unrelated path name),
   * then uses {@code edgeId} among endpoint matches, then falls back to name-only lookup.
   */
  public Optional<PlantPath> resolvePath(
      String vehicleEdgeId,
      String vehicleStartNodeId,
      String vehicleEndNodeId,
      NameResolver resolver
  ) {
    Objects.requireNonNull(resolver, "resolver");
    List<PlantPath> endpointMatches = new ArrayList<>();
    if (vehicleStartNodeId != null && vehicleEndNodeId != null) {
      String s = vehicleStartNodeId.trim();
      String e = vehicleEndNodeId.trim();
      for (String sn : resolver.pointKeys(s)) {
        for (String en : resolver.pointKeys(e)) {
          for (PlantPath pp : pathsByName.values()) {
            if (pp.sourcePointName().equals(sn) && pp.destinationPointName().equals(en)) {
              if (!endpointMatches.contains(pp)) {
                endpointMatches.add(pp);
              }
            }
          }
        }
      }
    }
    if (!endpointMatches.isEmpty()) {
      if (vehicleEdgeId != null && !vehicleEdgeId.isBlank()) {
        for (String key : resolver.pathKeys(vehicleEdgeId.trim())) {
          PlantPath named = pathsByName.get(key);
          if (named != null && endpointMatches.contains(named)) {
            return Optional.of(named);
          }
        }
      }
      return Optional.of(endpointMatches.get(0));
    }
    if (vehicleEdgeId != null && !vehicleEdgeId.isBlank()) {
      for (String key : resolver.pathKeys(vehicleEdgeId.trim())) {
        Optional<PlantPath> p = pathByName(key);
        if (p.isPresent()) {
          return p;
        }
      }
    }
    return Optional.empty();
  }

  /**
   * Builds candidate OpenTCS keys for matching VDA ids (raw, prefixed).
   */
  public static final class NameResolver {
    private final boolean applyStripping;
    private final String pointPrefix;
    private final String pathPrefix;

    /**
     * @param applyStripping when false, only the raw id is tried (same as AOS with stripping off).
     */
    public NameResolver(boolean applyStripping, String pointPrefix, String pathPrefix) {
      this.applyStripping = applyStripping;
      this.pointPrefix = pointPrefix != null ? pointPrefix : "";
      this.pathPrefix = pathPrefix != null ? pathPrefix : "";
    }

    public java.util.List<String> pointKeys(String vehicleId) {
      java.util.ArrayList<String> keys = new java.util.ArrayList<>();
      keys.add(vehicleId);
      if (applyStripping && !pointPrefix.isEmpty() && !vehicleId.startsWith(pointPrefix)) {
        keys.add(pointPrefix + vehicleId);
      }
      return keys;
    }

    public java.util.List<String> pathKeys(String vehicleId) {
      java.util.ArrayList<String> keys = new java.util.ArrayList<>();
      keys.add(vehicleId);
      if (applyStripping && !pathPrefix.isEmpty() && !vehicleId.startsWith(pathPrefix)) {
        keys.add(pathPrefix + vehicleId);
      }
      return keys;
    }
  }

  public static Builder builder(String modelName) {
    return new Builder(modelName);
  }

  public static final class Builder {
    private final String modelName;
    private final Map<String, PointMm> points = new HashMap<>();
    private final Map<String, PlantPath> paths = new HashMap<>();

    private Builder(String modelName) {
      this.modelName = modelName;
    }

    public Builder putPoint(String name, PointMm p) {
      points.put(name, p);
      return this;
    }

    public Builder putPath(String name, PlantPath p) {
      paths.put(name, p);
      return this;
    }

    public PlantModel build() {
      return new PlantModel(modelName, points, paths);
    }
  }
}
