package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449.auto.commands.RamseteControllerUnidirectionalDrive;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.function.Supplier;

public class ThreeBallHighCurvyAuto {
  public static final double MAX_CENTRIPETAL_ACCEL = 0.8;
  public static final Pose2d start = AutoUtils.pose(7.56, 2.99, 180 - 112.55);
  public static final Pose2d turnPoint = AutoUtils.pose(6.78, 1.93, 180 - 69.62);
  public static final Pose2d ball2 = AutoUtils.pose(5.54, 2.40, -132.80);
  public static final Pose2d between1 = AutoUtils.pose(5.97, 1.42, -13.02);
  public static final Pose2d ball3 = AutoUtils.pose(7.12, 1.00, -48.95);
  public static final double ballsEndVel = 3.0;
  public static final Pose2d between2 = AutoUtils.pose(7.99, 0.88, 41.01);
  public static final Pose2d between3 = AutoUtils.pose(7.93, 1.83, 98.13);
  public static final Pose2d end = AutoUtils.pose(7.93, 2.78, 78.93);

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      @NotNull Field2d field) {
    var constraints = List.of(new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCEL));
    var turnTraj =
        TrajectoryGenerator.generateTrajectory(
            List.of(start, turnPoint),
            trajConfig.get().setReversed(true).addConstraints(constraints));
    var getBallsTraj =
        TrajectoryGenerator.generateTrajectory(
            List.of(turnPoint, ball2, between1, ball3),
            trajConfig
                .get()
                .setReversed(false)
                .addConstraints(constraints)
                .setEndVelocity(ballsEndVel));
    var returnTraj =
        TrajectoryGenerator.generateTrajectory(
            List.of(ball3, between2 /*, between3*/, end),
            trajConfig
                .get()
                .setReversed(false)
                .addConstraints(constraints)
                .setStartVelocity(ballsEndVel));
    var fullTraj = turnTraj.concatenate(getBallsTraj).concatenate(returnTraj);
    field.getObject(ThreeBallHighCurvyAuto.class.getSimpleName()).setTrajectory(fullTraj);
    var totalTime = fullTraj.getTotalTimeSeconds();
    var deployWaitTime = 0.02;
    var retractWaitTime = turnTraj.getTotalTimeSeconds() + getBallsTraj.getTotalTimeSeconds() - deployWaitTime + 3.2;
    var shootWaitTime = totalTime - deployWaitTime - retractWaitTime - 1.5;
    return AutoUtils.shootHighCommand(cargo)
        .andThen(
            new RamseteControllerUnidirectionalDrive(drive, fullTraj)
                .alongWith(
                    new WaitCommand(deployWaitTime)
                        .andThen(cargo::deployIntake, cargo)
                        .andThen(cargo::runIntake, cargo)
                        .andThen(new WaitCommand(retractWaitTime))
                        .andThen(cargo::retractIntake, cargo)
                        .andThen(new WaitCommand(shootWaitTime))
                        .andThen(AutoUtils.shootHighCommand(cargo))));
  }
}
