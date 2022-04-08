package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.auto.commands.RamseteControllerUnidirectionalDrive;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.NavXTurnToAngle;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.function.Supplier;

public class ThreeBallHighStraightAuto {
  public static final Pose2d start = StationTwoBallHighStraightAuto.end;
  public static final Pose2d mid = AutoUtils.pose(7.47, 2.45, 75);
  public static final Pose2d midRev = AutoUtils.withAngle(mid, 190);
  public static final Pose2d ball = AutoUtils.pose(5.65, 1.98, -170.91);
  public static final Pose2d ballReversed = AutoUtils.withAngle(ball, -5);
  public static final Pose2d end = start;

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<PIDAngleController> angleController,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
    var reverseTraj =
        TrajectoryGenerator.generateTrajectory(
            List.of(start, mid), trajConfig.get().setReversed(true));
    field.getObject("ThreeBallHighStraightReverse").setTrajectory(reverseTraj);
    return StationTwoBallHighStraightAuto.createCommand(
            drive, cargo, angleController, trajConfig, field)
        .andThen(new RamseteControllerUnidirectionalDrive(drive, reverseTraj, false))
        .andThen(
            new NavXTurnToAngle(midRev.getRotation().getDegrees(), 3, drive, angleController.get()))
        .andThen(
            AutoUtils.getBallAndScoreHigh(
                drive,
                cargo,
                angleController.get(),
                trajConfig,
                List.of(midRev, ball),
                List.of(ballReversed, end),
                ThreeBallHighStraightAuto.class.getSimpleName(),
                field,
                false));
  }
}
