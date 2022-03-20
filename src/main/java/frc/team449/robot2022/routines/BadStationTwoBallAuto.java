package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team449.auto.builders.RamseteBuilder;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.function.Supplier;

public class BadStationTwoBallAuto {
  public static final Pose2d start =
      new Pose2d(7.89, 2.90, Rotation2d.fromDegrees(-113.88)); // TODO later
  public static final Pose2d ball =
      new Pose2d(6.69, 1.31, Rotation2d.fromDegrees(-171.42)); // 7.55, 1.06
  public static final Pose2d end =
          new Pose2d(4.67, 1.10, Rotation2d.fromDegrees(-141.70));

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull RamseteBuilder ramseteBuilder,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
    return new InstantCommand(cargo::deployIntake, cargo)
        .andThen(
            AutoUtils.getBallAndScore(
                cargo, ramseteBuilder, trajConfig, start, ball, end, "StationTwoBallAuto", field));
  }
}
