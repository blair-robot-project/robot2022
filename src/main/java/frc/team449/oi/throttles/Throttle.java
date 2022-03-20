package frc.team449.oi.throttles;

import frc.team449.updatable.Updatable;
import frc.team449.updatable.Updater;
import io.github.oblarg.oblog.annotations.Log;

/** An object representing an axis of a stick on a joystick. */
public abstract class Throttle implements Updatable, io.github.oblarg.oblog.Loggable {
  private double cachedValue;

  protected Throttle() {
    Updater.subscribe(this);
  }

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
