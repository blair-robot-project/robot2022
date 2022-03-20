package frc.team449.oi.throttles;

import org.jetbrains.annotations.NotNull;

import java.util.Set;

/** A Throttle that sums any number of other Throttles. */
public class ThrottleSum extends Throttle {

  /** The throttles to sum. */
  @NotNull private final Set<Throttle> throttles;

  /**
   * Default constructor.
   *
   * @param throttles The throttles to sum.
   */
  public ThrottleSum(@NotNull Set<Throttle> throttles) {
    this.throttles = throttles;
  }

  /** Sums the throttles and returns their output */
  @Override
  public double getValue() {
    // sum throttles
    double sum = 0;
    for (Throttle throttle : throttles) {
      sum += throttle.getValue();
    }

    // clip to [-1, 1]
    if (sum >= 1) {
      return 1;
    } else if (sum <= -1) {
      return -1;
    } else {
      return sum;
    }
  }

  /** Updates all cached values with current ones. */
  @Override
  public void update() {
    for (var throttle : this.throttles) {
      throttle.update();
    }
    super.update();
  }
}
