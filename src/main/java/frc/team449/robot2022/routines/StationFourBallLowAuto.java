package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.function.Supplier;

public class StationFourBallLowAuto {
  public static final Pose2d start = StationTwoBallLowAuto.end;
  public static final Pose2d ball3 = new Pose2d(5.57, 1.91, Rotation2d.fromDegrees(-171.63));
  public static final Pose2d ball4 = AutoUtils.pose(1.78, 1.24, -169.14);
  public static final Pose2d end = start;

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
    return StationTwoBallLowAuto.createCommand(drive, cargo, trajConfig, field)
        .andThen(
            AutoUtils.getBallAndScoreLow(
                drive,
                cargo,
                trajConfig,
                List.of(start, ball3, ball4),
                List.of(ball4, ball3, end),
                "StationFourBallAuto",
                field));
  }
}
