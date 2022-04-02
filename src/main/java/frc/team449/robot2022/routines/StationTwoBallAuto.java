package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.function.Supplier;

public class StationTwoBallAuto {
  public static final Pose2d start =
      new Pose2d(7.55, 1.84, Rotation2d.fromDegrees(-90)); // TODO later
  public static final Pose2d ball =
      new Pose2d(7.58, 0.85, Rotation2d.fromDegrees(-86)); // 7.55, 1.06
  public static final Pose2d end = AutoUtils.pose(7.80, 2.88, -110.82);
          //new Pose2d(7.60, 3.01, Rotation2d.fromDegrees(180 + 73));
  //          new Pose2d(7.53, 2.90, Rotation2d.fromDegrees(180 + 54.06));

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
    return new InstantCommand(cargo::deployIntake, cargo)
        .andThen(
            AutoUtils.getBallAndScore(
                drive,
                cargo,
                trajConfig,
                List.of(start, ball),
                List.of(ball, end),
                "StationTwoBallAuto",
                field));
  }
}
