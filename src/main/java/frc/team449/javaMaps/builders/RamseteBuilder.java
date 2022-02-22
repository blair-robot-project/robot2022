package frc.team449.javaMaps.builders;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Objects;

public final class RamseteBuilder {

  private @Nullable DriveUnidirectionalWithGyro drivetrain;
  private @Nullable PIDController leftPidController;
  private @Nullable PIDController rightPidController;
  private @Nullable Trajectory traj;
  private @Nullable Field2d field;
  private @Nullable String name;

  /** Set the drive subsystem which is to be controlled */
  public RamseteBuilder drivetrain(DriveUnidirectionalWithGyro drivetrain) {
    this.drivetrain = drivetrain;
    return this;
  }

  /** Set the PID controller for the left side. Uses velocity control. */
  public RamseteBuilder leftPidController(PIDController leftPidController) {
    this.leftPidController = leftPidController;
    return this;
  }

  /** Set the PID controller for the right side. Uses velocity control. */
  public RamseteBuilder rightPidController(PIDController rightPidController) {
    this.rightPidController = rightPidController;
    return this;
  }

  /** Set the trajectory for the robot to follow */
  public RamseteBuilder traj(Trajectory traj) {
    this.traj = traj;
    return this;
  }

  /**
   * Set the {@link Field2d} widget in Glass to display the trajectory on (optional)
   */
  public RamseteBuilder field(Field2d field) {
    this.field = field;
    return this;
  }

  /** Set the name to be displayed for this command and its trajectory if the field is given */
  public RamseteBuilder name(String name) {
    this.name = name;
    return this;
  }

  public RamseteBuilder copy() {
    return new RamseteBuilder()
        .drivetrain(this.drivetrain)
        .leftPidController(this.leftPidController)
        .rightPidController(this.rightPidController)
        .traj(traj)
        .field(field)
        .name(name);
  }

  public Command build() {
    assert drivetrain != null : "Drivetrain must not be null";
    assert leftPidController != null : "Left PID controller must not be null";
    assert rightPidController != null : "Right PID controller must not be null";
    assert traj != null : "Trajectory must not be null";

    if (field != null)
      field.getObject(Objects.requireNonNullElse(this.name, "traj")).setTrajectory(traj);

    var ramseteCmd =
        new RamseteCommand(
            this.traj,
            drivetrain::getCurrentPose,
            new RamseteController(),
            drivetrain.getFeedforward(),
            drivetrain.getDriveKinematics(),
            drivetrain::getWheelSpeeds,
            leftPidController,
            rightPidController,
            drivetrain::setVoltage,
            drivetrain);
    if (this.name != null) {
      ramseteCmd.setName(this.name);
    }
    return new InstantCommand(() -> drivetrain.resetOdometry(traj.getInitialPose()))
        .andThen(ramseteCmd)
        .andThen(() -> drivetrain.setVoltage(0, 0));
  }
}
