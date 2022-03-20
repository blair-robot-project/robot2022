package frc.team449.other;

import java.util.function.DoubleUnaryOperator;

/**
 * A component for limiting the second and first derivatives of a value. UNFINISHED DOESN'T ACTUALLY
 * WORK
 */
@Deprecated
public class SecondDerivativeLimit implements DoubleUnaryOperator {

  /** The maximum allowed change in the value per millisecond. */
  private final double firstDerivativeLimit;

  /** The maximum allowed change per millisecond of the change in the value per millisecond. */
  private final double secondDerivativeLimit;

  /** The value most recently received. */
  private double previousValue;

  /** The change in the value per millisecond the last time a value was received. */
  private double previousFirstDerivative;

  /** The time a value was most recently received */
  private double previousTime;

  /**
   * Default constructor.
   *
   * @param firstDerivativeLimit The maximum allowed change in the value per second.
   * @param secondDerivativeLimit The maximum allowed change per second of the change in the value
   *     per second.
   */
  public SecondDerivativeLimit(double firstDerivativeLimit, double secondDerivativeLimit) {
    this.firstDerivativeLimit = firstDerivativeLimit / 1000.; // convert to change/milliseconds
    this.secondDerivativeLimit =
        secondDerivativeLimit / 1000. / 1000.; // convert to change/millisecond^2
  }

  /**
   * Apply the derivative limiting to the given value.
   *
   * @param value The value to limit
   * @return The first and second derivative limited value
   */
  @Override
  public double applyAsDouble(double value) {
    // Time between the last value and this one
    double deltaTime = Clock.currentTimeMillis() - previousTime;
    if (value - previousValue > previousFirstDerivative) {
      // If the first derivative is increasing, make sure it's increasing slower than the
      // secondDerivativeLimit
      // and not exceeding the firstDerivativeLimit
      previousFirstDerivative =
          Math.min(
              previousFirstDerivative + deltaTime * secondDerivativeLimit,
              Math.min((value - previousValue) / deltaTime, firstDerivativeLimit));
    } else {
      // If the first derivative is decreasing, make sure it's decreasing slower than the
      // secondDerivativeLimit
      // and not going below the -firstDerivativeLimit
      previousFirstDerivative =
          Math.max(
              previousFirstDerivative - deltaTime * secondDerivativeLimit,
              Math.max((value - previousValue) / deltaTime, -firstDerivativeLimit));
    }
    previousValue = previousValue + previousFirstDerivative * deltaTime;
    previousTime = Clock.currentTimeMillis();
    return previousValue;
  }
}
