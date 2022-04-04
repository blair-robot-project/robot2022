package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.function.Supplier;

public class ThreeBallHighStraightAuto {
  public static final Pose2d start = StationTwoBallHighAuto.end;
  public static final Pose2d ball = ThreeBallLowAuto.ball;
  public static final Pose2d ballReversed = AutoUtils.reverse(ball);
  public static final Pose2d end = start;

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<PIDAngleController> angleController,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
    return StationTwoBallHighAuto.createCommand(drive, cargo, angleController, trajConfig, field)
        .andThen(
            AutoUtils.getBallAndScoreHigh(
                drive,
                cargo,
                angleController.get(),
                trajConfig,
                List.of(start, ball),
                List.of(ballReversed, end),
                ThreeBallHighStraightAuto.class.getSimpleName(),
                field));
  }
}
