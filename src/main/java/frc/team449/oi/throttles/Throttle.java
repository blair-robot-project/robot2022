package frc.team449.oi.throttles;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
import frc.team449.generalInterfaces.updatable.Updatable;
import io.github.oblarg.oblog.annotations.Log;

/** An object representing an axis of a stick on a joystick. */
@JsonTypeInfo(
    use = JsonTypeInfo.Id.CLASS,
    include = JsonTypeInfo.As.WRAPPER_OBJECT,
    property = "@class")
public abstract class Throttle implements Updatable, io.github.oblarg.oblog.Loggable {
  private double cachedValue;

  /**
   * Get the output of the throttle this object represents.
   *
   * @return The output from [-1, 1].
   */
  public abstract double getValue();

  /**
   * Get the cached output of the throttle this object represents.
   *
   * @return The output from [-1, 1].
   */
  @Log
  public final double getValueCached() {
    return cachedValue;
  }

  /** Updates all cached values with current ones. */
  @Override
  public void update() {
    cachedValue = getValue();
  }
}
