package frc.team449.oi.throttles;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleUnaryOperator;

/** A polynomial of a single variable. */
public class Polynomial implements DoubleUnaryOperator {

  /** A map of the powers and coefficients of each term. */
  @NotNull private final Map<Double, Double> powerToCoefficientMap;

  /**
   * Default constructor.
   *
   * @param powerToCoefficientMap A map of the powers and coefficients of each term. Defaults to
   *     [1:1] if null or 0-length.
   * @param scaleCoefficientSumTo Scales each coefficient so they all add up to this number. Can be
   *     null to avoid scaling.
   */
  public Polynomial(
      @Nullable Map<Double, Double> powerToCoefficientMap, @Nullable Double scaleCoefficientSumTo) {
    // Default powerToCoefficientMap to just be [1:1].
    if (powerToCoefficientMap == null || powerToCoefficientMap.size() == 0) {
      this.powerToCoefficientMap = new HashMap<>(1);
      this.powerToCoefficientMap.put(1., 1.);
    } else {
      // Make a copy in case the original Map is needed later or mutable
      this.powerToCoefficientMap = new HashMap<>(powerToCoefficientMap);
    }

    // Scale if scaleCoefficientSumTo isn't null.
    if (scaleCoefficientSumTo != null) {
      scaleCoefficientSum(scaleCoefficientSumTo);
    }
  }

  /**
   * Get the value of the polynomial given x.
   *
   * @param x The variable to be given to the polynomial.
   * @return The value of the polynomial evaluated at |x|, then changed to the sign of x.
   */
  @Override
  public double applyAsDouble(double x) {
    double sign = Math.signum(x);
    double abs = Math.abs(x);
    double toRet = 0;
    for (Map.Entry<Double, Double> power : powerToCoefficientMap.entrySet()) {
      toRet += Math.pow(abs, power.getKey()) * power.getValue();
    }
    return toRet * sign;
  }

  /**
   * Scale each coefficient so they sum to a given number.
   *
   * @param scaleTo The number to scale the sum of coefficients to.
   */
  public void scaleCoefficientSum(double scaleTo) {
    double coefficientSum =
        powerToCoefficientMap.values().stream().mapToDouble(Double::doubleValue).sum();
    double scaleFactor = scaleTo / coefficientSum;
    for (var powerCoef : powerToCoefficientMap.entrySet()) {
      powerToCoefficientMap.replace(powerCoef.getKey(), powerCoef.getValue() * scaleFactor);
    }
  }

  /** @return A map of the powers and coefficients of each term. */
  @NotNull
  public Map<Double, Double> getPowerToCoefficientMap() {
    return powerToCoefficientMap;
  }
}
