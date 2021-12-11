package frc.team449.generalInterfaces;

import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import frc.team449.generalInterfaces.shiftable.Shiftable;
import frc.team449.generalInterfaces.simpleMotor.SimpleMotor;
import frc.team449.jacksonWrappers.MappedSparkMaxBase;
import frc.team449.jacksonWrappers.simulated.MPSSmartMotorSimulated;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * A motor with built-in advanced capability featuring encoder, current limiting, and gear shifting
 * support. Also features built in MPS conversions.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public interface SmartMotor extends SimpleMotor, Shiftable, Loggable {
  /**
   * Set the motor output voltage to a given percent of available voltage.
   *
   * @param percentVoltage percent of total voltage from [-1, 1]
   */
  void setPercentVoltage(double percentVoltage);

  /**
   * Convert from native units read by an encoder to meters moved. Note this DOES account for
   * post-encoder gearing.
   *
   * @param nativeUnits A distance native units as measured by the encoder.
   * @return That distance in meters, or null if no encoder CPR was given.
   */
  double encoderToUnit(double nativeUnits);

  /**
   * Convert a distance from meters to encoder reading in native units. Note this DOES account for
   * post-encoder gearing.
   *
   * @param meters A distance in meters.
   * @return That distance in native units as measured by the encoder, or null if no encoder CPR was
   *     given.
   */
  double unitToEncoder(double meters);

  /**
   * Converts the velocity read by the controller's getVelocity() method to the MPS of the output
   * shaft. Note this DOES account for post-encoder gearing.
   *
   * @param encoderReading The velocity read from the encoder with no conversions.
   * @return The velocity of the output shaft, in MPS, when the encoder has that reading, or null if
   *     no encoder CPR was given.
   */
  double encoderToUPS(double encoderReading);

  /**
   * Converts from the velocity of the output shaft to what the controller's getVelocity() method
   * would read at that velocity. Note this DOES account for post-encoder gearing.
   *
   * @param mps The velocity of the output shaft, in MPS.
   * @return What the raw encoder reading would be at that velocity, or null if no encoder CPR was
   *     given.
   */
  double upsToEncoder(double mps);

  /**
   * Convert from native velocity units to output rotations per second. Note this DOES NOT account
   * for post-encoder gearing.
   *
   * @param nat A velocity in native units.
   * @return That velocity in RPS, or null if no encoder CPR was given.
   */
  Double nativeToRPS(double nat);

  /**
   * Convert from output RPS to the native velocity. Note this DOES NOT account for post-encoder
   * gearing.
   *
   * @param rps The RPS velocity you want to convert.
   * @return That velocity in native units, or null if no encoder CPR was given.
   */
  double rpsToNative(double rps);

  /** @return Raw position units for debugging purposes */
  double encoderPosition();

  /** Set a position setpoint for the controller. */
  void setPositionSetpoint(double meters);

  /**
   * Since the PID controller isn't accessible to this interface, this method must be overwritten to
   * set values for P, I, and D in {@link MappedSparkMaxBase#setGear(int gear)}
   */
  void setPID(double kP, double kI, double kD);

  /** @return Raw velocity units for debugging purposes */
  double encoderVelocity();

  /** Sets the output in volts. */
  void setVoltage(double volts);

  /**
   * Get the velocity of the controller in MPS.
   *
   * @return The controller's velocity in MPS, or null if no encoder CPR was given.
   */
  double getVelocity();

  /**
   * Give a velocity closed loop setpoint in MPS.
   *
   * @param velocity velocity setpoint in MPS.
   */
  void setVelocityUPS(double velocity);

  /**
   * Get the current closed-loop velocity error in MPS. WARNING: will give garbage if not in
   * velocity mode.
   *
   * @return The closed-loop error in MPS, or null if no encoder CPR was given.
   */
  double getError();

  /**
   * Get the current velocity setpoint of the motor in MPS, the position setpoint in meters
   *
   * @return The setpoint in sensible units for the current control mode.
   */
  double getSetpoint();

  /**
   * Get the voltage the motor is currently drawing from the PDP.
   *
   * @return Voltage in volts.
   */
  double getOutputVoltage();

  /**
   * Get the voltage available for the motor.
   *
   * @return Voltage in volts.
   */
  double getBatteryVoltage();

  /**
   * Get the current the motor is currently drawing from the PDP.
   *
   * @return Current in amps.
   */
  double getOutputCurrent();

  /**
   * Get the current control mode of the motor. Please don't use this for anything other than
   * logging.
   *
   * @return Control mode as a string.
   */
  String getControlMode();

  /**
   * Set the velocity scaled to a given gear's max velocity. Used mostly when autoshifting.
   *
   * @param velocity The velocity to go at, from [-1, 1], where 1 is the max speed of the given
   *     gear.
   * @param gear The number of the gear to use the max speed from to scale the velocity.
   */
  void setGearScaledVelocity(double velocity, int gear);

  /**
   * Set the velocity scaled to a given gear's max velocity. Used mostly when autoshifting.
   *
   * @param velocity The velocity to go at, from [-1, 1], where 1 is the max speed of the given
   *     gear.
   * @param gear The gear to use the max speed from to scale the velocity.
   */
  void setGearScaledVelocity(double velocity, Gear gear);

  /** @return Feedforward calculator for this gear */
  SimpleMotorFeedforward getCurrentGearFeedForward();

  /** @return the position of the motor in meters, or null of inches per rotation wasn't given. */
  double getPositionUnits();

  /** Resets the position of the motor to 0. */
  void resetPosition();

  /**
   * Get the status of the forwards limit switch.
   *
   * @return True if the forwards limit switch is closed, false if it's open or doesn't exist.
   */
  boolean isFwdLimitSwitch();

  /**
   * Get the status of the reverse limit switch.
   *
   * @return True if the reverse limit switch is closed, false if it's open or doesn't exist.
   */
  boolean isRevLimitSwitch();

  boolean isInhibitedForward();

  boolean isInhibitedReverse();

  /**
   * Gets the CAN port of this controller.
   *
   * @return the CAN port of this controller
   */
  int getPort();

  @Override
  default LayoutType configureLayoutType() {
    return BuiltInLayouts.kGrid;
  }

  /**
   * Gets the name of this instance of the class.
   *
   * @return the name of this instance when logging
   */
  @Override
  String configureLogName();

  /**
   * Gets the default width and height of the layout of this instance of the class in Shuffleboard.
   * f
   *
   * @return an array of {width, height}.
   */
  @Override
  default int[] configureLayoutSize() {
    return new int[] {4, 3};
  }

  @Override
  default int[] configureLayoutPosition() {
    return new int[] {3, 3};
  }

  /**
   * Gets whether the motor is a simulated motor.
   *
   * @return whether the motor is a software simulation of a motor
   */
  @Log
  default boolean isSimulated() {
    return false;
  }

  enum Type {
    /** RevRobotics SPARK MAX */
    SPARK("SparkMax"),
    /** CTRE Talon SRX */
    TALON("Talon"),
    /**
     * Simulated motor
     *
     * @see MPSSmartMotorSimulated
     */
    SIMULATED("SIMULATED");

    public final String friendlyName;

    Type(final String friendlyName) {
      this.friendlyName = friendlyName;
    }

    @Override
    public String toString() {
      return friendlyName;
    }
  }
}
