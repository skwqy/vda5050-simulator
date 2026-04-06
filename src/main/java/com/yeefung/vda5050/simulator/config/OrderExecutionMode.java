package com.yeefung.vda5050.simulator.config;

/**
 * How {@code order.edges} are turned into motion segments — see {@code docs/订单边执行模式设计.md}.
 */
public enum OrderExecutionMode {
  /** Use all edges in {@code sequenceId} order; ignore {@code released}. */
  IGNORE_RELEASED,
  /** Only the maximal leading prefix where each edge is {@code released} (see config for missing field). */
  STRICT_PREFIX_RELEASED
}
