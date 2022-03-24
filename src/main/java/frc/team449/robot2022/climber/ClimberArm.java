package frc.team449.robot2022.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.motor.WrappedMotor;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

import java.util.function.BooleanSupplier;

/**
 * A single arm of the 2022 climber. Periodically checks if the hall sensor is active. If it is,
 * then it determines whether the arm is at the bottom or mid climb height limit, and resets its
 * motor's encoder's position based on that
 */
public class ClimberArm extends SubsystemBase implements Loggable {
  private final @NotNull WrappedMotor motor;
  private final @NotNull BooleanSupplier hallSensor;
  private final double sensorDifferentiationHeight;
  private final double midClimbLimit;
  private final double topLimit;
  private double prevOutput = 0.0;
  @Log.ToString private @NotNull ClimberArm.ClimberState state = ClimberState.BOTTOM;

  /**
   * @param motor Winch motor used for retracting/extending arm
   * @param sensorDifferentiationHeight If the arm is above this height and the hall sensor is on,
   *     the arm is considered to be at the mid climb height limit. If the arm is below this height
   *     and the hall effect sensor is on, the arm is considered to be at the bottom
   * @param midClimbLimit The height limit for mid climb
   * @param topLimit The distance to the hard stop
   * @param hallSensor Hall effect sensor to determine if the climber is at either the bottom or mid
   *     climb height limit
   */
  public ClimberArm(
      @NotNull WrappedMotor motor,
      double sensorDifferentiationHeight,
      double midClimbLimit,
      double topLimit,
      @NotNull BooleanSupplier hallSensor) {
    this.motor = motor;
    this.sensorDifferentiationHeight = sensorDifferentiationHeight;
    this.midClimbLimit = midClimbLimit;
    this.topLimit = topLimit;
    this.hallSensor = hallSensor;
  }

  /** Whether this arm's reached the bottom. */
  @Log
  public boolean reachedBottom() {
    return state == ClimberState.BOTTOM;
  }

  /**
   * Whether the arm is below the given state
   */
  public boolean belowState(@NotNull ClimberState state) {
    return this.state.compareTo(state) < 0;
  }

  @Override
  public void periodic() {
    var sensorOn = this.sensorOn();
    switch (this.state) {
      case BOTTOM:
        if (!sensorOn) {
          this.state = ClimberState.BELOW_MID;
        }
        break;
      case BELOW_MID:
        if (sensorOn) {
          this.state = prevOutput < 0 ? ClimberState.BOTTOM : ClimberState.MID_LIMIT;
        }
        break;
      case MID_LIMIT:
        if (!sensorOn) {
          this.state = prevOutput < 0 ? ClimberState.BELOW_MID : ClimberState.ABOVE_MID;
        }
        break;
      case ABOVE_MID:
        if (sensorOn) {
          this.state = prevOutput < 0 ? ClimberState.MID_LIMIT : ClimberState.TOP;
        }
        break;
      case TOP:
        if (!sensorOn) {
          this.state = ClimberState.ABOVE_MID;
        }
    }

    // Reset encoder position based on the Hall effect sensors
    if (this.state == ClimberState.BOTTOM) {
      motor.encoder.resetPosition(0);
    } else if (this.state == ClimberState.MID_LIMIT) {
      motor.encoder.resetPosition(midClimbLimit);
    } else if (this.state == ClimberState.TOP) {
      motor.encoder.resetPosition(topLimit);
    }
  }

  public void set(double output) {
    this.motor.set(output);
    this.prevOutput = output;
  }

  public double getHeight() {
    return motor.getPositionUnits();
  }

  @Override
  public String configureLogName() {
    return "ClimberArm" + motor.configureLogName();
  }

  /**
   * Inverted because WPILib is stupid
   */
  @Log
  private boolean sensorOn() {
    return !this.hallSensor.getAsBoolean();
  }

  public enum ClimberState {
    /** Bottom position */
    BOTTOM,
    /** Between the bottom and mid climb limit */
    BELOW_MID,
    /** Mid climb height limit */
    MID_LIMIT,
    /**
     * Somewhere above the mid climb height limit (should only be when doing high climb (climber is
     * not stowed))
     */
    ABOVE_MID,
    /** At the height limit for high/traversal climb */
    TOP
  }
}
