package frc.team449.oi.throttles;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
import frc.team449.generalInterfaces.updatable.Updatable;

/** An object representing an axis of a stick on a joystick. */
@JsonTypeInfo(
    use = JsonTypeInfo.Id.CLASS,
    include = JsonTypeInfo.As.WRAPPER_OBJECT,
    property = "@class")
public interface Throttle extends Updatable, io.github.oblarg.oblog.Loggable {

  /**
   * Get the output of the throttle this object represents.
   *
   * @return The output from [-1, 1].
   */
  double getValue();

  /**
   * Get the cached output of the throttle this object represents.
   *
   * @return The output from [-1, 1].
   */
  double getValueCached();
}
