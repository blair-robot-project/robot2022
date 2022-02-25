package frc.team449.generalInterfaces;

import com.fasterxml.jackson.annotation.JsonCreator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * An object representing the settings for a drivetrain. (previously {@code
 * Shiftable.PerGearSettings})
 */
public class DriveSettings {
  /** The ramp rate, in volts/sec. null means no ramp rate. */
  @Nullable public final Double rampRate;
  /** The maximum speed of the motor in this gear, in MPS. Used for throttle scaling. */
  @Nullable public final Double maxSpeed;
  /** Feedforward calculator */
  public final SimpleMotorFeedforward feedforward;
  /** Position PID controller for left side */
  public final PIDController leftPosPID;
  /** Position PID controller for right side */
  public final PIDController rightPosPID;
  /** Velocity PID controller for left side */
  public final PIDController leftVelPID;
  /** Velocity PID controller for right side */
  public final PIDController rightVelPID;
  /** The track width of the robot/distance between left and right wheels in meters */
  public final double trackWidth;

  /**
   * Default constructor.
   *
   * @param feedforward The component for calculating feedforwards for the left side in
   *     closed-loop control modes.
   * @param leftPosPID Left position PID controller
   * @param rightPosPID Right position PID controller
   * @param leftVelPID Left velocity PID controller
   * @param rightVelPID Right velocity PID controller
   * @param rampRate The ramp rate, in volts/sec. Can be null, and if it is, no ramp rate is used.
   * @param maxSpeed The maximum speed of the motor in this gear, in MPS. Used for throttle scaling.
   * @param trackWidth The distance between the left and right wheels in meters
   */
  @JsonCreator
  public DriveSettings(
      @NotNull SimpleMotorFeedforward feedforward,
      @NotNull PIDController leftPosPID,
      @NotNull PIDController rightPosPID,
      @NotNull PIDController leftVelPID,
      @NotNull PIDController rightVelPID,
      @Nullable Double rampRate,
      @Nullable Double maxSpeed,
      double trackWidth) {
    this.feedforward = feedforward;
    this.leftPosPID = leftPosPID;
    this.rightPosPID = rightPosPID;
    this.leftVelPID = leftVelPID;
    this.rightVelPID = rightVelPID;
    this.rampRate = rampRate;
    this.maxSpeed = maxSpeed;
    this.trackWidth = trackWidth;
  }
}
