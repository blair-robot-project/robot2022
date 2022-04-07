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

public class FiveBallLowAuto {
  // Carry over the two ball auto and append to it another path that forms the three ball auto
  public static final Pose2d start = ThreeBallLowAuto.end;
  public static final Pose2d ball = StationFourBallLowAuto.ball4;
//      new Pose2d(1.64, 1.15, Rotation2d.fromDegrees(-169.14));
  public static final Pose2d end = start; // new Pose2d(7.60, 2.98, Rotation2d.fromDegrees(-109.86));

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
    return ThreeBallLowAuto.createCommand(drive, cargo, trajConfig, field)
        .andThen(
            AutoUtils.getBallAndScoreLow(
                drive,
                cargo,
                trajConfig,
                List.of(start, ball),
                List.of(ball, end),
                "FiveBallAuto",
                field));
  }
}
