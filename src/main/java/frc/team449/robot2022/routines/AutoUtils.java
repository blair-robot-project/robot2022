package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.auto.commands.RamseteControllerUnidirectionalDrive;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.NavXTurnToAngle;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.List;
import java.util.function.Supplier;

/** Helpers for auto */
public final class AutoUtils {
  private AutoUtils() {}

  public static Pose2d pose(double x, double y, double degs) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(degs));
  }

  /** Return a pose with the same x, y coords but a different heading (degrees) */
  public static Pose2d withAngle(Pose2d pose, double degs) {
    return pose(pose.getX(), pose.getY(), degs);
  }

  /** Reverse the heading of a pose */
  public static Pose2d reverse(Pose2d pose) {
    return withAngle(pose, pose.getRotation().getDegrees() + 180);
  }

  /**
   * Create an auto command that runs intake, goes to a ball, picks it up, comes back, and spits it
   * out. Despite the name, the robot may already have a preloaded ball xor may happen to pick up
   * two balls on the way
   *
   * @param toBallTraj The trajectory to go to the ball
   * @param fromBallTraj The trajectory to go back to the hub
   */
  public static Command getBallAndScoreLow(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Trajectory toBallTraj,
      @NotNull Trajectory fromBallTraj,
      String name,
      @Nullable Field2d field) {
    var fullTraj = toBallTraj.concatenate(fromBallTraj);

    if (field != null) {
      field.getObject(name).setTrajectory(fullTraj);
    }

    // How much to wait to spit after the intake starts
    var spitWaitTime =
        fullTraj.getTotalTimeSeconds()
            - AutoConstants.PAUSE_BEFORE_INTAKE
            - AutoConstants.PAUSE_AFTER_SPIT;
    return new RamseteControllerUnidirectionalDrive(drive, fullTraj)
        .alongWith(
            new InstantCommand(cargo::removeHood),
            new WaitCommand(AutoConstants.PAUSE_BEFORE_INTAKE)
                .andThen(cargo::runIntake, cargo)
                .andThen(new WaitCommand(spitWaitTime))
                .andThen(cargo.startShooterCommand()));
  }

  /**
   * Create an auto command that runs intake, goes to a ball, picks it up, comes back, and spits it
   * out. Despite the name, the robot may already have a preloaded ball xor may happen to pick up
   * two balls on the way
   *
   * @param toBall The poses to hit on the way to the ball (including the start and end poses)
   * @param fromBall The poses to hit on the way back to the hub (including the start and end poses)
   */
  public static Command getBallAndScoreLow(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      @NotNull List<Pose2d> toBall,
      @NotNull List<Pose2d> fromBall,
      String name,
      @Nullable Field2d field) {
    var toBallTraj =
        TrajectoryGenerator.generateTrajectory(toBall, trajConfig.get().setReversed(false));
    var fromBallTraj =
        TrajectoryGenerator.generateTrajectory(fromBall, trajConfig.get().setReversed(true));
    return AutoUtils.getBallAndScoreLow(drive, cargo, toBallTraj, fromBallTraj, name, field);
  }

  /**
   * Create an auto command that runs intake, goes to a ball, picks it up, comes back, and spits it
   * out. Despite the name, the robot may already have a preloaded ball xor may happen to pick up
   * two balls on the way
   *
   * @param toBall The points to hit on the way to the ball (including the start and end poses)
   * @param fromBall The points to hit on the way back to the hub (including the start and end
   *     poses)
   */
  public static Command getBallAndScoreVectors(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      @NotNull List<Spline.ControlVector> toBall,
      @NotNull List<Spline.ControlVector> fromBall,
      String name,
      @Nullable Field2d field) {
    var toBallTraj =
        TrajectoryGenerator.generateTrajectory(
            new TrajectoryGenerator.ControlVectorList(toBall), trajConfig.get().setReversed(false));
    var fromBallTraj =
        TrajectoryGenerator.generateTrajectory(
            new TrajectoryGenerator.ControlVectorList(fromBall),
            trajConfig.get().setReversed(true));
    return AutoUtils.getBallAndScoreLow(drive, cargo, toBallTraj, fromBallTraj, name, field);
  }

  /**
   * Create an auto command that runs intake, goes to a ball, picks it up, turns around, comes back,
   * and shoots it into the upper hub. Despite the name, the robot may already have a preloaded ball
   * xor may happen to pick up two balls on the way.<br>
   * Assumes you start facing the ball. The Command ends with the robot facing the hub
   *
   * @param toBall The poses to hit on the way to the ball (including the start and end poses)
   * @param fromBall The poses to hit on the way back to the hub (including the start and end poses)
   */
  public static Command getBallAndScoreHigh(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull PIDAngleController angleController,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      @NotNull List<Pose2d> toBall,
      @NotNull List<Pose2d> fromBall,
      String name,
      @Nullable Field2d field) {
    var toBallTraj =
        TrajectoryGenerator.generateTrajectory(toBall, trajConfig.get().setReversed(false));
    var fromBallTraj =
        TrajectoryGenerator.generateTrajectory(fromBall, trajConfig.get().setReversed(false));

    if (field != null) {
      field.getObject(name + "toball").setTrajectory(toBallTraj);
      field.getObject(name + "fromball").setTrajectory(fromBallTraj);
    }

    SmartDashboard.putData(
        "PIDAngle",
        builder -> {
          builder.addDoubleProperty("setpoint", angleController::getSetpoint, x -> {});
          builder.addDoubleProperty(
              "actual", () -> angleController.getSetpoint() + angleController.getError(), x -> {});
        });

    var shootWaitTime = fromBallTraj.getTotalTimeSeconds() - AutoConstants.SHOOT_HEADSTART;

    return new InstantCommand(cargo::deployIntake, cargo)
        .andThen(cargo::runIntake, cargo)
        .andThen(new RamseteControllerUnidirectionalDrive(drive, toBallTraj))
        .andThen(
            new NavXTurnToAngle(
                fromBall.get(0).getRotation().getDegrees(),
                AutoConstants.TURN_TIMEOUT,
                drive,
                angleController))
        .andThen(
            new RamseteControllerUnidirectionalDrive(drive, fromBallTraj)
                .alongWith(
                    new InstantCommand(cargo::deployHood, cargo)
                        .andThen(cargo::retractIntake, cargo)
                        .andThen(cargo::runIntake, cargo)
                        .andThen(new WaitCommand(AutoConstants.SHOOT_HEADSTART))
                        .andThen(AutoUtils.shootHighCommand(cargo))));
  }

  /** A complete command to shoot high */
  public static Command shootHighCommand(@NotNull Cargo2022 cargo) {
    return new InstantCommand(cargo::deployHood)
        .andThen(cargo.startShooterCommand())
        .andThen(new WaitCommand(AutoConstants.PAUSE_AFTER_SHOOT))
        .andThen(cargo::stop, cargo);
  }
}
