package frc.team449.oi.throttles;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import frc.team449.other.Updater;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

import java.util.Set;

/** A Throttle that sums any number of other Throttles. */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class ThrottleSum implements Throttle {

  /** The throttles to sum. */
  @NotNull private final Set<Throttle> throttles;
  /** The cached output. */
  private double cachedValue;

  /**
   * Default constructor.
   *
   * @param throttles The throttles to sum.
   */
  @JsonCreator
  public ThrottleSum(@NotNull @JsonProperty(required = true) Set<Throttle> throttles) {
    this.throttles = throttles;
    Updater.subscribe(this);
  }

  /** Sums the throttles and returns their output */
  @Override
  public double getValue() {
    var sum = throttles.stream().mapToDouble(Throttle::getValue).sum();
    return sum / throttles.size();
  }

  /**
   * Get the cached output of the throttle this object represents.
   *
   * @return The output from [-1, 1].
   */
  @Override
  @Log
  public double getValueCached() {
    return cachedValue;
  }

  /** Updates all cached values with current ones. */
  @Override
  public void update() {
    cachedValue = getValue();
  }
}
