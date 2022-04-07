package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.function.Supplier;

public class StationTwoBallHighStraightAuto {

  public static final Pose2d start = StationTwoBallLowAuto.start;
  public static final Pose2d ball = StationTwoBallLowAuto.ball;
  public static final Pose2d ballReversed = AutoUtils.reverse(ball);
  public static final Pose2d end = AutoUtils.pose(7.63, 2.89, 77.47);

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<PIDAngleController> angleController,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
    return new InstantCommand(cargo::deployIntake, cargo)
        .andThen(
            AutoUtils.getBallAndScoreHigh(
                drive,
                cargo,
                angleController.get(),
                trajConfig,
                List.of(start, ball),
                List.of(ballReversed, end),
                StationTwoBallHighStraightAuto.class.getSimpleName(),
                field));
  }
}
