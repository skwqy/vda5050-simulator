package com.yeefung.vda5050.simulator.util;

import com.fasterxml.jackson.databind.JsonNode;

/** Small helpers for reading fields from Jackson {@link JsonNode} trees. */
public final class JsonNodes {

  private JsonNodes() {}

  public static String text(JsonNode n, String field) {
    JsonNode v = n.get(field);
    return v != null && v.isTextual() ? v.asText() : null;
  }

  public static long longVal(JsonNode n, String field) {
    JsonNode v = n.get(field);
    if (v == null || v.isNull()) {
      return 0L;
    }
    return v.asLong();
  }

  public static Double doubleVal(JsonNode n, String field) {
    JsonNode v = n.get(field);
    if (v == null || v.isNull()) {
      return null;
    }
    return v.asDouble();
  }

  public static double doubleOr(JsonNode e, String field, double def) {
    Double v = doubleVal(e, field);
    return v != null && v > 0 ? v : def;
  }
}
