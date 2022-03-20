package frc.team449.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * An object representing the settings for a drivetrain. (previously {@code
 * Shiftable.PerGearSettings})
 */
public class DriveSettings {
  /** Feedforward calculator */
  public final SimpleMotorFeedforward feedforward;
  /** Velocity PID controller for left side */
  public final PIDController leftVelPID;
  /** Velocity PID controller for right side */
  public final PIDController rightVelPID;
  /** The track width of the robot/distance between left and right wheels in meters */
  public final double trackWidth;

  /**
   * Default constructor.
   *
   * @param feedforward The component for calculating feedforwards for the left side in closed-loop
   *     control modes.
   * @param leftVelPID Left velocity PID controller
   * @param rightVelPID Right velocity PID controller
   * @param trackWidth The distance between the left and right wheels in meters
   */
  public DriveSettings(
      @NotNull SimpleMotorFeedforward feedforward,
      @NotNull PIDController leftVelPID,
      @NotNull PIDController rightVelPID,
      double trackWidth) {
    this.feedforward = feedforward;
    this.leftVelPID = leftVelPID;
    this.rightVelPID = rightVelPID;
    this.trackWidth = trackWidth;
  }
}
