package frc.team449.oi.throttles;

import frc.team449.generalInterfaces.doubleUnaryOperator.RampComponent;
import org.jetbrains.annotations.NotNull;

public class ThrottleWithRamp extends Throttle {

  private final Throttle throttle;
  private final RampComponent ramp;

  public ThrottleWithRamp(@NotNull Throttle throttle, @NotNull RampComponent ramp) {
    this.throttle = throttle;
    this.ramp = ramp;
  }

  @Override
  public double getValue() {
    return ramp.applyAsDouble(throttle.getValue());
  }
}
