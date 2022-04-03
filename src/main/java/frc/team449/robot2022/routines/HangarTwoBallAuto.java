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

public class HangarTwoBallAuto {
  public static final Pose2d start =
      new Pose2d(6.06, 5.13, Rotation2d.fromDegrees(135)); // TODO later
  public static final Pose2d ball = new Pose2d(5.33, 5.87, Rotation2d.fromDegrees(132.44));
  public static final Pose2d end = new Pose2d(6.99, 4.53, Rotation2d.fromDegrees(180 - 21.64));

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
    return new InstantCommand(cargo::deployIntake, cargo)
        .andThen(
            AutoUtils.getBallAndScoreLow(
                drive,
                cargo,
                trajConfig,
                List.of(start, ball),
                List.of(ball, end),
                "HangarTwoBallAuto",
                field));
  }
}
