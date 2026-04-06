package com.yeefung.vda5050.simulator.mqtt;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.yeefung.vda5050.simulator.config.SimulatorProperties;
import com.yeefung.vda5050.simulator.core.SimulationEngine;
import com.yeefung.vda5050.simulator.core.vda.VdaHeaderIds;
import com.yeefung.vda5050.simulator.core.vda.VdaMessageBuilder;
import java.nio.charset.StandardCharsets;
import java.util.Objects;
import java.util.concurrent.Executor;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;

/**
 * Subscribes to {@code order} / {@code instantActions}; publishes {@code connection}, {@code state},
 * {@code visualization} — aligned with YFAOS {@code CommAdapterImpl} enable() subscriptions.
 */
public final class MqttVdaBridge implements AutoCloseable {

  private static final System.Logger LOG = System.getLogger(MqttVdaBridge.class.getName());

  private final SimulatorProperties props;
  private final SimulationEngine engine;
  private final ObjectMapper json;
  private final HeaderClock clock;
  private final Executor executor;
  private final VdaMessageBuilder vdaMessageBuilder;

  private MqttClient client;

  public MqttVdaBridge(
      SimulatorProperties props,
      SimulationEngine engine,
      ObjectMapper json,
      HeaderClock clock,
      Executor executor,
      VdaMessageBuilder vdaMessageBuilder
  ) {
    this.props = Objects.requireNonNull(props, "props");
    this.engine = Objects.requireNonNull(engine, "engine");
    this.json = Objects.requireNonNull(json, "json");
    this.clock = Objects.requireNonNull(clock, "clock");
    this.executor = Objects.requireNonNull(executor, "executor");
    this.vdaMessageBuilder = Objects.requireNonNull(vdaMessageBuilder, "vdaMessageBuilder");
  }

  public void start() throws Exception {
    String prefix = props.topicPrefix();
    client = new MqttClient(props.mqtt.brokerUri, props.mqtt.clientId, new MemoryPersistence());
    MqttConnectOptions opt = new MqttConnectOptions();
    opt.setAutomaticReconnect(true);
    opt.setCleanSession(true);
    if (props.mqtt.username != null && !props.mqtt.username.isBlank()) {
      opt.setUserName(props.mqtt.username);
      if (props.mqtt.password != null) {
        opt.setPassword(props.mqtt.password.toCharArray());
      }
    }

    client.setCallback(new MqttCallback() {
      @Override
      public void connectionLost(Throwable cause) {
        LOG.log(System.Logger.Level.WARNING, "MQTT connection lost: " + cause, cause);
      }

      @Override
      public void messageArrived(String topic, MqttMessage message) {
        byte[] payload = message.getPayload();
        executor.execute(() -> {
          try {
            if (topic.endsWith("/order")) {
              engine.onOrderMessage(json.readTree(payload));
              publishStateIfDue();
            } else if (topic.endsWith("/instantActions")) {
              engine.onInstantActionsMessage(json.readTree(payload));
              publishStateIfDue();
            }
          } catch (Exception e) {
            LOG.log(System.Logger.Level.ERROR, "Failed to handle MQTT on " + topic, e);
          }
        });
      }

      @Override
      public void deliveryComplete(IMqttDeliveryToken token) {
        // optional
      }
    });

    client.connect(opt);
    client.subscribe(prefix + "/order", props.mqtt.subscribe.order);
    client.subscribe(prefix + "/instantActions", props.mqtt.subscribe.instantActions);
    publishConnectionOnline();
    // Initial state so AOS sees ONLINE + pose and lastNodeId (e.g. from initialPointName) immediately
    publishState();
    LOG.log(System.Logger.Level.INFO, "Simulator connected; topicPrefix=" + prefix);
  }

  public void publishConnectionOnline() throws Exception {
    String prefix = props.topicPrefix();
    var root = vdaMessageBuilder.buildConnection(props, new VdaHeaderIds(clock.nextConnection()));
    byte[] bytes = json.writeValueAsBytes(root);
    String topic = prefix + "/connection";
    logOutbound("connection", topic, bytes);
    client.publish(topic, bytes, props.mqtt.publish.connection, false);
  }

  public void publishState() {
    if (client == null || !client.isConnected()) {
      return;
    }
    try {
      long hid = clock.nextState();
      var node = engine.buildState(new VdaHeaderIds(hid));
      byte[] bytes = json.writeValueAsBytes(node);
      String topic = props.topicPrefix() + "/state";
      logOutbound("state", topic, bytes);
      client.publish(topic, bytes, props.mqtt.publish.state, false);
      engine.afterStatePublished();
    } catch (Exception e) {
      LOG.log(System.Logger.Level.ERROR, "publishState failed", e);
    }
  }

  /** Publishes {@code state} only when the engine says a point-related update is due. */
  public void publishStateIfDue() {
    if (engine.shouldPublishState()) {
      publishState();
    }
  }

  public void publishVisualization() {
    if (client == null || !client.isConnected()) {
      return;
    }
    try {
      long hid = clock.nextVisualization();
      var node = engine.buildVisualization(new VdaHeaderIds(hid));
      byte[] bytes = json.writeValueAsBytes(node);
      String topic = props.topicPrefix() + "/visualization";
      logOutbound("visualization", topic, bytes);
      client.publish(topic, bytes, props.mqtt.publish.visualization, false);
    } catch (Exception e) {
      LOG.log(System.Logger.Level.ERROR, "publishVisualization failed", e);
    }
  }

  private static void logOutbound(String label, String topic, byte[] payload) {
    System.out.println(
        "[VDA5050 OUT] " + label + " topic=" + topic + " payload=" + new String(payload, StandardCharsets.UTF_8));
  }

  @Override
  public void close() {
    if (client != null && client.isConnected()) {
      try {
        client.disconnect();
      } catch (MqttException e) {
        LOG.log(System.Logger.Level.DEBUG, "disconnect", e);
      }
    }
  }
}
