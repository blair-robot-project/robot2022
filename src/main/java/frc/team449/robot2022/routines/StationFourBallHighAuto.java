package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class StationFourBallHighAuto {
  public static final Pose2d start = StationTwoBallHighStraightAuto.end;
  public static final Pose2d midTurn = ThreeBallHighStraightAuto.mid;
  public static final Pose2d midTurnRev = ThreeBallHighStraightAuto.midRev;
  public static final Pose2d ball3 = new Pose2d(5.57, 1.91, Rotation2d.fromDegrees(-171.63));
  public static final Pose2d ball4 =
      AutoUtils.withAngle(
          StationFourBallLowAuto.ball4,
          -169.14); // new Pose2d(1.69, 1.15, Rotation2d.fromDegrees(180.0));
  public static final Pose2d ball4Rev = AutoUtils.withAngle(ball4, -5);
  public static final Pose2d end = start;

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<PIDAngleController> angleController,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
    var reverseTraj =
        TrajectoryGenerator.generateTrajectory(
            List.of(start, midTurn), trajConfig.get().setReversed(true));
    field.getObject("StationFourBallHighReverse").setTrajectory(reverseTraj);
    return StationTwoBallHighStraightAuto.createCommand(
            drive, cargo, angleController, trajConfig, field)
        .andThen(new RamseteControllerUnidirectionalDrive(drive, reverseTraj, false))
        .andThen(
            new NavXTurnToAngle(
                midTurnRev.getRotation().getDegrees(), 1, drive, angleController.get()))
        .andThen(
            AutoUtils.getBallAndScoreHigh(
                drive,
                cargo,
                angleController.get(),
                trajConfig,
                List.of(midTurnRev, ball3, ball4),
                List.of(ball4Rev, end),
                "StationFourBallAuto",
                field));
  }
}
