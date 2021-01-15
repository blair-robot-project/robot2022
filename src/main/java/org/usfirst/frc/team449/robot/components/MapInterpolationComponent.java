package org.usfirst.frc.team449.robot.components;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import java.util.AbstractMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class MapInterpolationComponent {

  /**
   * Linear or cosine, until more functionality is added
   */
  private InterpolationMethod currentMethod;

  /**
   * LookUpTable, the table of experimentally optimized values
   */
  private TreeMap<Double, Double> LUT;

  /**
   * Upper and lower limits of an interpolation calc
   */
  private Map.Entry<Double, Double> upper;
  private Map.Entry<Double, Double> lower;

  /**
   * Default constructor
   * @param method the interpolation method
   * @param entries the list of experimentally derived values for the LUT
   */
  @JsonCreator
  public MapInterpolationComponent(
      @JsonProperty(required = true) InterpolationMethod method,
      @JsonProperty(required = true) List<Map.Entry<Double, Double>> entries) {
    currentMethod = method;
    LUT = new TreeMap<>();
    for (Map.Entry<Double, Double> entry : entries) {
      LUT.put(entry.getKey(), entry.getValue());
    }
  }

  /**
   * Changes the interpolation method
   */
  public void updateMethod(InterpolationMethod method) {
    currentMethod = method;
  }

  /**
   * Calculates the appropriate value from distance x
   * @param x the distance from the target
   * @return the shooter velocity from distance x
   */
  public double calculate(double x) {
    if (LUT.containsKey(x)) {
      return LUT.get(x);
    }
    updateEntries(x);
    double ratio;
    try {
      ratio = (x - lower.getKey()) / (upper.getKey() - lower.getKey());
    } catch (ArithmeticException e) {
      ratio = 0;
    }
    switch (currentMethod) {
      case LINEAR:
        return linear(ratio);
      case COSINE:
        return cosine(ratio);
      default:
        return 0;
    }
  }

  /**
   * Sets the upper and lower limits of the next interpolation
   * @param x the distance from the target
   */
  private void updateEntries(double x) {
    lower = LUT.floorEntry(x) != null ? LUT.floorEntry(x) : new AbstractMap.SimpleEntry<>(0., 0.);
    upper =
        LUT.ceilingEntry(x) != null ? LUT.ceilingEntry(x) : new AbstractMap.SimpleEntry<>(0., 0.);
  }

  /**
   * Linear interpolation method
   * @param x the distance from the target
   * @return the shooter vel
   */
  private double linear(double x) {
    return lower.getValue() * (1 - x) + upper.getValue() * x;
  }

  /**
   * Cosine interpolation method
   * @param x the distance from the target
   * @return the shooter vel
   */
  private double cosine(double x) {
    double smoothpoint = (1 - Math.cos(x * Math.PI)) / 2;
    return linear(smoothpoint);
  }

  // http://paulbourke.net/miscellaneous/interpolation/
  enum InterpolationMethod {
    LINEAR,
    COSINE
  }
}
