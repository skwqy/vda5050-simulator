package com.yeefung.vda5050.simulator.config;

import java.util.Objects;

/**
 * Loaded from {@code application.yaml}; mirrors YFAOS {@code CommAdapterImpl} topic prefix rules.
 */
public class SimulatorProperties {

  public Mqtt mqtt = new Mqtt();
  public Simulation simulation = new Simulation();
  /** Optional OpenTCS plant model (driving course XML) for curve-accurate motion. */
  public Map map = new Map();

  public String topicPrefix() {
    return mqtt.interfaceName
        + "/v" + mqtt.majorVersion
        + "/" + mqtt.manufacturer
        + "/" + mqtt.serialNumber;
  }

  public static final class Mqtt {
    public String brokerUri = "tcp://127.0.0.1:1883";
    public String clientId = "vda5050-sim";
    public String username;
    public String password;
    public String interfaceName = "defaultInterface";
    public int majorVersion = 2;
    public String manufacturer = "defaultManufacturer";
    public String serialNumber = "SIM001";
    public Publish publish = new Publish();
    public Subscribe subscribe = new Subscribe();

    public static final class Publish {
      public int connection = 1;
      public int state = 0;
      public int visualization = 0;
    }

    public static final class Subscribe {
      public int order = 1;
      public int instantActions = 1;
    }
  }

  public static final class Simulation {
    /**
     * Physics simulation step (ms). Does <b>not</b> set {@code state} period — state is sent only on
     * new point ({@code lastNodeId} / sequence change); use {@link #visualizationMinIntervalMs} for vis.
     */
    public int stateIntervalMs = 100;
    /** Minimum interval between {@code visualization} messages (ms). */
    public int visualizationMinIntervalMs = 500;
    public double defaultSpeedMps = 0.5;
    public double arrivalEpsilonM = 0.05;
    public String protocolVersion = "2.1.0";
    public String operatingMode = "AUTOMATIC";
    /**
     * OpenTCS point name for initial pose (e.g. {@code Point_1}). Requires {@code map.xmlPath} loaded;
     * sets {@code agvPosition} and {@code lastNodeId} before any order. Empty = (0,0) and empty lastNodeId.
     */
    public String initialPointName = "";
  }

  /**
   * OpenTCS 6 driving course XML (e.g. exported from Model Editor). Control-point layout uses the
   * same transform as YFAOS {@code AgvCommunicationAdapter}: x_mm = layoutX * layoutScaleMm,
   * y_mm = layoutFlipY ? -layoutY * layoutScaleMm : layoutY * layoutScaleMm.
   */
  public static final class Map {
    /** Absolute path to plant XML, or null to use only VDA order node positions (straight segments). */
    public String xmlPath;
    /** VDA {@code agvPosition.mapId}; defaults to model name from XML when loaded. */
    public String mapId;
    /**
     * Same as {@code aos.vda5050.map-name-prefixes.apply-stripping}: when true, orders use stripped
     * node/edge ids and plant XML uses prefixed OpenTCS names (prefix strings below).
     */
    public boolean applyStripping = true;
    /** Same as {@code yeefungagv.plc.name-prefixes.point} — OpenTCS point name prefix. */
    public String pointPrefix = "Point_";
    /** Same as {@code yeefungagv.plc.name-prefixes.path} — OpenTCS path name prefix. */
    public String pathPrefix = "Path_";
    /** Layout unit to millimetres (OpenTCS editor default aligns with 50 in AgvCommunicationAdapter). */
    public double layoutScaleMm = 50.0;
    /** If true, layout Y maps to world Y as {@code -y * layoutScaleMm} (matches AgvCommunicationAdapter). */
    public boolean layoutFlipY = true;
  }

  public void validate() {
    Objects.requireNonNull(mqtt.brokerUri, "mqtt.brokerUri");
    Objects.requireNonNull(mqtt.clientId, "mqtt.clientId");
    if (mqtt.majorVersion < 1) {
      throw new IllegalArgumentException("mqtt.majorVersion must be >= 1");
    }
  }
}
