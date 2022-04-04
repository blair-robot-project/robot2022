package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
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
  private static final Pose2d start = AutoUtils.pose(7.56, 2.99, 180 - 112.55);
  private static final Pose2d turnPoint = AutoUtils.pose(6.91, 1.81, 180 - 111.47);
  private static final Pose2d ball2 = AutoUtils.pose(5.54, 2.3, -140.28);
  private static final Pose2d between = AutoUtils.pose(5.75, 1.13, -37.50);
  private static final Pose2d ball3 = AutoUtils.pose(7.13, 0.66, -5.00);
  private static final Pose2d between2 = AutoUtils.pose(8.03, 1.16, 82.87);
  public static final Pose2d end = start;
  public static final double MAX_CENTRIPETAL_ACCEL = 0.8;

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      @NotNull Field2d field) {
    var constraints =
        List.of(
            new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCEL));
    var turnTraj =
        TrajectoryGenerator.generateTrajectory(
            List.of(start, turnPoint),
            trajConfig.get().setReversed(true).addConstraints(constraints));
    var getBallsTraj =
        TrajectoryGenerator.generateTrajectory(
            List.of(turnPoint, ball2, between, ball3, between2, end),
            trajConfig.get().setReversed(false).addConstraints(constraints));
    var fullTraj = turnTraj.concatenate(getBallsTraj);
    field.getObject(ThreeBallLowAuto.class.getSimpleName()).setTrajectory(fullTraj);
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
