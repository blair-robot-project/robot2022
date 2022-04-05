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
  private static final Pose2d start = AutoUtils.pose(7.56, 2.99, 180 - 112.55);
  public static final Pose2d end = start;
  private static final Pose2d turnPoint = AutoUtils.pose(6.83, 2.26, 180 - 79.38);
  private static final Pose2d ball2 = AutoUtils.pose(5.52, 2.40, -129.29);
  private static final Pose2d between1 = AutoUtils.pose(5.98, 1.27, -34.08);
  private static final Pose2d ball3 = AutoUtils.pose(7.26, 0.83, -48.95);
  private static final Pose2d between2 = AutoUtils.pose(8.11, 0.85, 67.89);
  private static final Pose2d between3 = AutoUtils.pose(8.23, 1.75, 98.13);

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
            List.of(turnPoint, ball2, between1, ball3, between2, between3, end),
            trajConfig.get().setReversed(false).addConstraints(constraints));
    var fullTraj = turnTraj.concatenate(getBallsTraj);
    field.getObject(ThreeBallHighCurvyAuto.class.getSimpleName()).setTrajectory(fullTraj);
    return AutoUtils.shootHighCommand(cargo)
        .andThen(
            new RamseteControllerUnidirectionalDrive(drive, fullTraj)
                .alongWith(
                    new WaitCommand(0.2)
                        .andThen(cargo::deployIntake, cargo)
                        .andThen(cargo::runIntake, cargo)
                        .andThen(new WaitCommand(fullTraj.getTotalTimeSeconds() - 3))
                        .andThen(cargo::retractIntake, cargo)))
        .andThen(AutoUtils.shootHighCommand(cargo));
  }
}
