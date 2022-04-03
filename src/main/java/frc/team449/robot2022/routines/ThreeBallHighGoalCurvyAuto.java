package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team449.auto.commands.RamseteControllerUnidirectionalDrive;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.function.Supplier;

public class ThreeBallHighGoalCurvyAuto {
  private static final Pose2d start = AutoUtils.pose(7.58, 2.97, 180 - 112.55);
  private static final Pose2d turnPoint = AutoUtils.pose(7.30, 1.23, 180 - 79.53);
  private static final Pose2d ball2 = AutoUtils.pose(5.58, 2.16, -151.70);
  private static final Pose2d between = AutoUtils.pose(5.65, 0.77, -17.61);
  private static final Pose2d ball3 = AutoUtils.pose(7.23, 0.50, 0);
  public static final pose2d end = AutoUtils.pose(7.58, 2.97, -112.55);

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      @NotNull Field2d field) {
    var turnTraj = TrajectoryGenerator.generateTrajectory(
            List.of(start, turnPoint), trajConfig.get().setReversed(true));
    var getBallsTraj =
        TrajectoryGenerator.generateTrajectory(
            List.of(turnPoint, ball2, between, ball3, start), trajConfig.get().setReversed(true));
    var returnTraj =
        TrajectoryGenerator.generateTrajectory(List.of(ball3, start), trajConfig.get());
    var fullTraj = getBallsTraj.concatenate(returnTraj);
    field.getObject(ThreeBallAuto.class.getSimpleName()).setTrajectory(fullTraj);
    return AutoUtils.shootHighCommand(cargo)
        .andThen(cargo::runIntake, cargo)
        .andThen(new RamseteControllerUnidirectionalDrive(drive, fullTraj))
        .andThen(AutoUtils.shootHighCommand(cargo));
  }
}
