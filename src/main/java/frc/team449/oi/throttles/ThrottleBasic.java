package frc.team449.oi.throttles;

import edu.wpi.first.wpilibj.GenericHID;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

/** A class representing a single axis on a joystick. */
public class ThrottleBasic extends Throttle {

  /** The stick we're using */
  @NotNull protected final GenericHID stick;

  /** The axis on the joystick we care about. */
  private final int axis;

  /** Whether or not the controls should be inverted */
  private final boolean inverted;

  /**
   * Default constructor.
   *
   * @param stick The Joystick object being used
   * @param axis The axis being used. 0 is X, 1 is Y, 2 is Z.
   * @param inverted Whether or not to invert the joystick input. Defaults to false.
   */
  public ThrottleBasic(@NotNull final GenericHID stick, final int axis, final boolean inverted) {
    this.stick = stick;
    this.axis = axis;
    this.inverted = inverted;
  }

  /**
   * Gets the raw value from the stick and inverts it if necessary.
   *
   * @return The raw joystick output, on [-1, 1].
   */
  @Log
  @Override
  public double getValue() {
    return (inverted ? -1 : 1) * stick.getRawAxis(axis);
  }

  /**
   * Get the result to use in PIDController.
   *
   * @return the result to use in PIDController
   */
  @Log
  public double pidGet() {
    return (inverted ? -1 : 1) * stick.getRawAxis(axis);
  }

  @Override
  public String configureLogName() {
    return "Throttle " + stick.getName() + " axis " + axis;
  }
}
