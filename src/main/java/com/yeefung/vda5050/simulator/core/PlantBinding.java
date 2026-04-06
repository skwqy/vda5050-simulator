package com.yeefung.vda5050.simulator.core;

import com.yeefung.vda5050.simulator.config.SimulatorProperties;
import com.yeefung.vda5050.simulator.map.LayoutTransform;
import com.yeefung.vda5050.simulator.map.PlantModel;
import com.yeefung.vda5050.simulator.map.Vda5050MapNameCodec;

/**
 * Shared plant/map name wiring for {@link SimulationEngine} and {@link com.yeefung.vda5050.simulator.core.order.OrderRoutePlanner}
 * so codec and resolver instances are not duplicated.
 */
public record PlantBinding(
    LayoutTransform layoutTransform,
    Vda5050MapNameCodec mapNameCodec,
    PlantModel.NameResolver nameResolver
) {
  public static PlantBinding from(SimulatorProperties props, PlantModel plantModel) {
    LayoutTransform layoutTransform =
        plantModel != null ? new LayoutTransform(props.map) : null;
    Vda5050MapNameCodec mapNameCodec =
        new Vda5050MapNameCodec(
            props.map.applyStripping, props.map.pointPrefix, props.map.pathPrefix);
    PlantModel.NameResolver nameResolver =
        new PlantModel.NameResolver(
            props.map.applyStripping, props.map.pointPrefix, props.map.pathPrefix);
    return new PlantBinding(layoutTransform, mapNameCodec, nameResolver);
  }
}
