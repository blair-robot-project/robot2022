package frc.team449.oi.unidirectional.tank;

import frc.team449.oi.throttles.Throttle;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

/** A simple tank drive, where each joystick controls a side of the robot. */
public class OITankSimple extends OITank {

  /** The left throttle */
  @NotNull private final Throttle leftThrottle;

  /** The right throttle */
  @NotNull private final Throttle rightThrottle;

  /**
   * The difference between left and right input within which the driver is considered to be trying
   * to drive straight.
   */
  private final double commandingStraightTolerance;

  /**
   * Default constructor
   *
   * @param leftThrottle The throttle for controlling the velocity of the left side of the drive.
   * @param rightThrottle The throttle for controlling the velocity of the right side of the drive.
   * @param commandingStraightTolerance The difference between left and right input within which the
   *     driver is considered to be trying to drive straight. Defaults to 0.
   */
  public OITankSimple(
      @NotNull final Throttle leftThrottle,
      @NotNull final Throttle rightThrottle,
      final double commandingStraightTolerance) {
    this.leftThrottle = leftThrottle;
    this.rightThrottle = rightThrottle;
    this.commandingStraightTolerance = commandingStraightTolerance;
  }

  /**
   * Get the throttle for the left side of the drive.
   *
   * @return percent of max speed for left motor cluster from [-1.0, 1.0]
   */
  @Override
  @Log
  public double getLeftThrottle() {
    // If the driver is trying to drive straight, use the average of the two sticks.
    if (commandingStraight()) {
      return (leftThrottle.getValue() + rightThrottle.getValue()) / 2.;
    }
    return leftThrottle.getValue();
  }

  /**
   * Get the throttle for the right side of the drive.
   *
   * @return percent of max speed for right motor cluster from [-1.0, 1.0]
   */
  @Override
  @Log
  public double getRightThrottle() {
    // If the driver is trying to drive straight, use the average of the two sticks.
    if (commandingStraight()) {
      return (leftThrottle.getValue() + rightThrottle.getValue()) / 2.;
    }
    return rightThrottle.getValue();
  }

  /**
   * Whether the driver is trying to drive straight.
   *
   * @return True if the driver is trying to drive straight, false otherwise.
   */
  @Override
  @Log
  public boolean commandingStraight() {
    return Math.abs(getLeftOutputCached() - getRightOutputCached()) <= commandingStraightTolerance;
  }
}
