package frc.team449.auto.builders;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.auto.commands.RamseteControllerUnidirectionalDrive;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.NavXTurnToAngle;
import org.jetbrains.annotations.Nullable;

import java.util.Objects;

public final class RamseteBuilder {

  private @Nullable DriveUnidirectionalWithGyro drivetrain;
  private @Nullable Trajectory traj;
  private @Nullable Field2d field;
  private @Nullable String name;
  private @Nullable PIDAngleController pidAngleController;
  private double angleTimeout = 0;

  /** Set the drive subsystem which is to be controlled */
  public RamseteBuilder drivetrain(DriveUnidirectionalWithGyro drivetrain) {
    this.drivetrain = drivetrain;
    return this;
  }

  /** Set the trajectory for the robot to follow */
  public RamseteBuilder traj(Trajectory traj) {
    this.traj = traj;
    return this;
  }

  /** Set the {@link Field2d} widget in Glass to display the trajectory on (optional) */
  public RamseteBuilder field(Field2d field) {
    this.field = field;
    return this;
  }

  /** Set the name to be displayed for this command and its trajectory if the field is given */
  public RamseteBuilder name(String name) {
    this.name = name;
    return this;
  }

  /**
   * Set the PIDAngleController to use when turning the robot in place at the end of the trajectory.
   * The controller is optional
   */
  public RamseteBuilder anglePID(PIDAngleController controller) {
    this.pidAngleController = controller;
    return this;
  }

  /** The number of seconds until the angle command after the actual Ramsete command times out */
  public RamseteBuilder angleTimeout(double timeout) {
    this.angleTimeout = timeout;
    return this;
  }

  public RamseteBuilder copy() {
    return new RamseteBuilder()
        .drivetrain(drivetrain)
        .traj(traj)
        .field(field)
        .name(name)
        .anglePID(pidAngleController)
        .angleTimeout(angleTimeout);
  }

  public Command build() {
    Objects.requireNonNull(drivetrain, "Drivetrain must not be null");
    Objects.requireNonNull(traj, "Trajectory must not be null");

    if (field != null)
      field.getObject(Objects.requireNonNullElse(this.name, "traj")).setTrajectory(traj);

    var ramseteCmd =
        new RamseteControllerUnidirectionalDrive(
            drivetrain,
            drivetrain.leftVelPID(),
            drivetrain.rightVelPID(),
            traj,
            drivetrain.getFeedforward());
    if (this.name != null) {
      ramseteCmd.setName(this.name);
    }

    var lastPose = traj.getStates().get(traj.getStates().size() - 1).poseMeters;

    var cmd =
        new InstantCommand(() -> drivetrain.resetOdometry(traj.getInitialPose()))
            .andThen(ramseteCmd)
            .andThen(drivetrain::fullStop, drivetrain);
    cmd.setName("Ramsete command");

    // If angleTimeout is nonzero, then we want to turn after the Ramsete command is over
    // to ensure our heading is right
    if (angleTimeout > 0) {
      Objects.requireNonNull(
          pidAngleController, "PIDAngleController must not be null if turning after Ramsete");
      cmd =
          cmd.andThen(
                  new NavXTurnToAngle(
                      lastPose.getRotation().getDegrees(),
                      angleTimeout,
                      drivetrain,
                      pidAngleController))
              .andThen(drivetrain::fullStop, drivetrain);
      cmd.setName("Ramsete command with NavX");
    }
    return cmd;
  }
}
