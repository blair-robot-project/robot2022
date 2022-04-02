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

public class ThreeBallAuto {
  // Carry over the two ball auto and append to it another path that forms the three ball auto
  public static final Pose2d start = StationTwoBallAuto.end;
  public static final Pose2d ball =
      new Pose2d(5.41, 1.76, Rotation2d.fromDegrees(133.92)); // 5.54, 2.24, 360 - 146.61
  // new Pose2d(5.54, 2.06, Rotation2d.fromDegrees(-169.14));
  public static final Pose2d end = start;

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
    return StationTwoBallAuto.createCommand(drive, cargo, trajConfig, field)
        .andThen(
            AutoUtils.getBallAndScore(
                drive,
                cargo,
                trajConfig,
                List.of(start, ball),
                List.of(ball, end),
                "ThreeBallAuto",
                field));
  }
}
