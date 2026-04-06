package com.yeefung.vda5050.simulator.mqtt;

import java.util.concurrent.atomic.AtomicLong;

/**
 * VDA5050: headerId is per topic, monotonically increasing (see YFAOS CommAdapterImpl.sendMessage).
 */
public final class HeaderClock {

  private final AtomicLong connection = new AtomicLong();
  private final AtomicLong state = new AtomicLong();
  private final AtomicLong visualization = new AtomicLong();

  public long nextConnection() {
    return connection.getAndIncrement();
  }

  public long nextState() {
    return state.getAndIncrement();
  }

  public long nextVisualization() {
    return visualization.getAndIncrement();
  }
}
