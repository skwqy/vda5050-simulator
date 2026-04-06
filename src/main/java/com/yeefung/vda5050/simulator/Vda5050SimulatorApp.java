package com.yeefung.vda5050.simulator;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.fasterxml.jackson.datatype.jsr310.JavaTimeModule;
import com.yeefung.vda5050.simulator.config.SimulatorProperties;
import com.yeefung.vda5050.simulator.core.PlantBinding;
import com.yeefung.vda5050.simulator.core.SimulationEngine;
import com.yeefung.vda5050.simulator.core.order.OrderRoutePlanner;
import com.yeefung.vda5050.simulator.core.vda.VdaMessageBuilder;
import com.yeefung.vda5050.simulator.map.OpenTcsPlantXmlLoader;
import com.yeefung.vda5050.simulator.map.PlantModel;
import com.yeefung.vda5050.simulator.mqtt.HeaderClock;
import com.yeefung.vda5050.simulator.mqtt.MqttVdaBridge;
import java.io.InputStream;
import java.nio.file.Path;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Standalone VDA5050 vehicle simulator for YFAOS MQTT integration tests.
 *
 * <p>Config: classpath {@code application.yaml}, or path {@code -Dsimulator.config=/path/to.yaml}.
 */
public final class Vda5050SimulatorApp {

  private Vda5050SimulatorApp() {}

  public static void main(String[] args) throws Exception {
    SimulatorProperties props = loadProperties();
    props.validate();

    ObjectMapper json = new ObjectMapper();
    json.registerModule(new JavaTimeModule());

    PlantModel plantModel = loadPlantModel(props);
    PlantBinding plantBinding = PlantBinding.from(props, plantModel);
    VdaMessageBuilder vdaMessageBuilder = new VdaMessageBuilder();
    OrderRoutePlanner orderRoutePlanner =
        new OrderRoutePlanner(
            plantModel,
            plantBinding.layoutTransform(),
            plantBinding.nameResolver(),
            plantBinding.mapNameCodec());
    var engine =
        new SimulationEngine(props, plantModel, vdaMessageBuilder, orderRoutePlanner, plantBinding);
    var clock = new HeaderClock();
    var mqttExecutor = Executors.newSingleThreadExecutor(r -> {
      Thread t = new Thread(r, "mqtt-order");
      t.setDaemon(true);
      return t;
    });

    var bridge = new MqttVdaBridge(props, engine, json, clock, mqttExecutor, vdaMessageBuilder);
    bridge.start();

    ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor(r -> {
      Thread t = new Thread(r, "sim-tick");
      t.setDaemon(true);
      return t;
    });

    long tickMs = props.simulation.stateIntervalMs;
    double dt = tickMs / 1000.0;
    long visMin = props.simulation.visualizationMinIntervalMs;
    AtomicLong lastVis = new AtomicLong(System.nanoTime());

    scheduler.scheduleAtFixedRate(() -> {
      try {
        engine.tick(dt);
        bridge.publishStateIfDue();
        long now = System.nanoTime();
        if (TimeUnit.NANOSECONDS.toMillis(now - lastVis.get()) >= visMin) {
          lastVis.set(now);
          bridge.publishVisualization();
        }
      } catch (Exception e) {
        System.getLogger(Vda5050SimulatorApp.class.getName()).log(
            System.Logger.Level.ERROR, "simulation tick failed", e);
      }
    }, 0, tickMs, TimeUnit.MILLISECONDS);

    CountDownLatch stop = new CountDownLatch(1);
    Runtime.getRuntime().addShutdownHook(new Thread(() -> {
      scheduler.shutdown();
      bridge.close();
      mqttExecutor.shutdown();
      stop.countDown();
    }));

    System.out.println("VDA5050 simulator running. topicPrefix=" + props.topicPrefix()
        + " broker=" + props.mqtt.brokerUri
        + (plantModel != null ? " plant=" + plantModel.getModelName() : "")
        + " (Ctrl+C to stop)");
    stop.await();
  }

  private static PlantModel loadPlantModel(SimulatorProperties props) {
    String p = props.map.xmlPath;
    if (p == null || p.isBlank()) {
      return null;
    }
    try {
      return OpenTcsPlantXmlLoader.load(Path.of(p.trim()));
    } catch (Exception e) {
      System.getLogger(Vda5050SimulatorApp.class.getName()).log(
          System.Logger.Level.ERROR,
          "Failed to load plant XML: " + p + " — falling back to straight-line motion",
          e
      );
      return null;
    }
  }

  private static SimulatorProperties loadProperties() throws Exception {
    String path = System.getProperty("simulator.config");
    ObjectMapper yamlMapper = new ObjectMapper(new YAMLFactory());
    yamlMapper.registerModule(new JavaTimeModule());
    if (path != null && !path.isBlank()) {
      return yamlMapper.readValue(Path.of(path).toFile(), SimulatorProperties.class);
    }
    try (InputStream in = Vda5050SimulatorApp.class.getClassLoader()
        .getResourceAsStream("application.yaml")) {
      if (in == null) {
        throw new IllegalStateException("application.yaml not on classpath");
      }
      return yamlMapper.readValue(in, SimulatorProperties.class);
    }
  }
}
