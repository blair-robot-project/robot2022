package frc.team449.generalInterfaces;

import com.fasterxml.jackson.annotation.JsonCreator;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
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
  /**
   * The coefficient the output changes by after being measured by the encoder, e.g. this would be
   * 1/70 if there was a 70:1 gearing between the encoder and the final output.
   */
  public final double postEncoderGearing;
  /** Feedforward calculator for left side */
  public final SimpleMotorFeedforward leftFeedforward;
  /** Feedforward calculator for right side */
  public final SimpleMotorFeedforward rightFeedforward;
  /** Position PID controller for left side */
  public final PIDController leftPosPID;
  /** Position PID controller for right side */
  public final PIDController rightPosPID;
  /** Velocity PID controller for left side */
  public final PIDController leftVelPID;
  /** Velocity PID controller for right side */
  public final PIDController rightVelPID;

  /**
   * Default constructor.
   *
   * @param leftFeedforward The component for calculating feedforwards for the left side in
   *     closed-loop control modes.
   * @param rightFeedforward The component for calculating feedforwards for the right side in
   *     closed-loop control modes.
   * @param leftPosPID Left position PID controller
   * @param rightPosPID Right position PID controller
   * @param leftVelPID Left velocity PID controller
   * @param rightVelPID Right velocity PID controller
   * @param rampRate The ramp rate, in volts/sec. Can be null, and if it is, no ramp rate is used.
   * @param maxSpeed The maximum speed of the motor in this gear, in MPS. Used for throttle scaling.
   * @param postEncoderGearing The coefficient the output changes by after being measured by the
   *     encoder
   */
  @JsonCreator
  public DriveSettings(
      @NotNull SimpleMotorFeedforward leftFeedforward,
      @NotNull SimpleMotorFeedforward rightFeedforward,
      @NotNull PIDController leftPosPID,
      @NotNull PIDController rightPosPID,
      @NotNull PIDController leftVelPID,
      @NotNull PIDController rightVelPID,
      @Nullable Double rampRate,
      @Nullable Double maxSpeed,
      double postEncoderGearing) {
    this.leftFeedforward = leftFeedforward;
    this.rightFeedforward = rightFeedforward;
    this.leftPosPID = leftPosPID;
    this.rightPosPID = rightPosPID;
    this.leftVelPID = leftVelPID;
    this.rightVelPID = rightVelPID;
    this.rampRate = rampRate;
    this.postEncoderGearing = postEncoderGearing;
    this.maxSpeed = maxSpeed;
  }

  /** Empty constructor that uses all default options. */
  public DriveSettings() {
    this(
        new SimpleMotorFeedforward(0, 0),
        new SimpleMotorFeedforward(0, 0),
        new PIDController(0, 0, 0),
        new PIDController(0, 0, 0),
        new PIDController(0, 0, 0),
        new PIDController(0, 0, 0),
        null,
        null,
        1.0);
  }
}