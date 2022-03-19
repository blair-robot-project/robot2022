package frc.team449.oi;

import com.fasterxml.jackson.annotation.JsonCreator;
import frc.team449.other.Clock;
import java.util.function.DoubleUnaryOperator;
import org.jetbrains.annotations.NotNull;

/** A component for limiting the rate of change of a value. Mainly used for limiting acceleration of drive */
public class RampComponent implements DoubleUnaryOperator {

  /** The maximum allowed change in the value per second. */
  private final double maxIncreasePerMillis, maxDecreasePerMillis;

  /** The value most recently returned. */
  private double lastValue;

  /** The time, in milliseconds, that the value most recently returned was returned at. */
  private long lastTime;

  /**
   *
   * @param maxIncreasePerSecond The maximum allowed increase in the value per second.
   * @param maxDecreasePerSecond The maximum allowed decrease in the value per second. Should be
   *     positive. Defaults to maxIncreasePerSecond.
   */
  @JsonCreator
  public RampComponent(
      double maxIncreasePerSecond,
      double maxDecreasePerSecond) {
    this.maxIncreasePerMillis = maxIncreasePerSecond / 1000.;
    this.maxDecreasePerMillis = maxDecreasePerSecond / 1000.;
  }

  /**
   * {@link RampComponent#maxDecreasePerMillis} is taken to be the same as {@link RampComponent#maxIncreasePerMillis}
   *
   * @param maxChangePerSecond The maximum allowed increase/decrease in the value per second.
   */
  @JsonCreator
  public RampComponent(double maxChangePerSecond) {
    this(maxChangePerSecond, maxChangePerSecond);
  }

  /**
   * Ramp the given value.
   *
   * @param value The current value of whatever it is you're ramping
   * @return The ramped version of that value.
   */
  @Override
  public double applyAsDouble(double value) {
    if (value > lastValue) {
      lastValue =
          Math.min(
              value, lastValue + (Clock.currentTimeMillis() - lastTime) * maxIncreasePerMillis);
    } else {
      lastValue =
          Math.max(
              value, lastValue - (Clock.currentTimeMillis() - lastTime) * maxDecreasePerMillis);
    }
    lastTime = Clock.currentTimeMillis();
    return lastValue;
  }

  /**
   * Get an a copy of this object.
   *
   * @return a new {@link RampComponent} with the same max change per second
   */
  @NotNull
  public RampComponent copy() {
    return new RampComponent(maxIncreasePerMillis * 1000., maxDecreasePerMillis * 1000.);
  }
}
