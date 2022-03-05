package frc.team449.auto.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.AHRS.NavXTurnToAngle;
import org.jetbrains.annotations.Nullable;

import java.util.Objects;

public final class RamseteBuilder {

  private @Nullable DriveUnidirectionalWithGyro drivetrain;
  private @Nullable PIDController leftPid;
  private @Nullable PIDController rightPid;
  private @Nullable Trajectory traj;
  private @Nullable Field2d field;
  private @Nullable String name;
  private double b = 2;
  private double zeta = .7;
  private PIDAngleController pidAngleController;
  private double angleTimeout = 0;

  /** Set the drive subsystem which is to be controlled */
  public RamseteBuilder drivetrain(DriveUnidirectionalWithGyro drivetrain) {
    this.drivetrain = drivetrain;
    return this;
  }

  /** Set the PID controller for the left side. Uses velocity control. */
  public RamseteBuilder leftPid(PIDController leftPid) {
    this.leftPid = leftPid;
    return this;
  }

  /** Set the PID controller for the right side. Uses velocity control. */
  public RamseteBuilder rightPid(PIDController rightPid) {
    this.rightPid = rightPid;
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
   * Set the b parameter for Ramsete (b &gt; 0 rad²/m²) for which larger values make convergence
   * more aggressive like a proportional term.
   */
  public RamseteBuilder b(double b) {
    this.b = b;
    return this;
  }

  /**
   * Set the zeta tuning parameter for Ramsete (0 rad<sup>-1</sup> &lt; zeta &lt; 1
   * rad<sup>-1</sup>) for which larger values provide more damping in response.
   */
  public RamseteBuilder zeta(double zeta) {
    this.zeta = zeta;
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
        .drivetrain(this.drivetrain)
        .leftPid(this.leftPid)
        .rightPid(this.rightPid)
        .traj(traj)
        .field(field)
        .name(name)
        .b(b)
        .zeta(zeta)
        .anglePID(pidAngleController)
        .angleTimeout(angleTimeout);
  }

  public Command build() {
    assert drivetrain != null : "Drivetrain must not be null";
    assert leftPid != null : "Left PID controller must not be null";
    assert rightPid != null : "Right PID controller must not be null";
    assert traj != null : "Trajectory must not be null";

    SmartDashboard.putData(
        "ramsete pid controllers",
        (builder) -> {
          builder.addDoubleProperty("leftpid", leftPid::getSetpoint, x -> {});
          builder.addDoubleProperty("rightpid", rightPid::getSetpoint, x -> {});
          builder.addDoubleProperty("lefterr", leftPid::getPositionError, x -> {});
          builder.addDoubleProperty("righterr", rightPid::getPositionError, x -> {});
          builder.addDoubleProperty(
              "leftvel", () -> leftPid.getSetpoint() - leftPid.getPositionError(), x -> {});
          builder.addDoubleProperty(
              "rightvel", () -> rightPid.getSetpoint() - rightPid.getPositionError(), x -> {});
        });
    if (field != null)
      field.getObject(Objects.requireNonNullElse(this.name, "traj")).setTrajectory(traj);

    var ramseteCmd =
        new RamseteCommand(
            this.traj,
            drivetrain::getCurrentPose,
            new RamseteController(b, zeta),
            drivetrain.getFeedforward(),
            drivetrain.getDriveKinematics(),
            drivetrain::getWheelSpeeds,
            leftPid,
            rightPid,
            drivetrain::setVoltage,
            drivetrain);
    if (this.name != null) {
      ramseteCmd.setName(this.name);
    }

    var lastPose = traj.getStates().get(traj.getStates().size() - 1).poseMeters;

    return new InstantCommand(() -> drivetrain.resetOdometry(traj.getInitialPose()))
        .andThen(ramseteCmd)
//        .andThen(() -> drivetrain.setVoltage(0, 0))
//        .andThen(
//            new NavXTurnToAngle<>(
//                lastPose.getRotation().getDegrees(), angleTimeout, drivetrain, pidAngleController))
        .andThen(() -> drivetrain.setVoltage(0, 0));
  }
}
