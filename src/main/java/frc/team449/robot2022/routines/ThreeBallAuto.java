package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449.auto.builders.RamseteBuilder;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.function.Supplier;

public class ThreeBallAuto {
  // Carry over the two ball auto and append to it another path that forms the three ball auto
  public static Pose2d start = StationTwoBallAuto.end;
  public static Pose2d ball =
      new Pose2d(5.41, 1.76, Rotation2d.fromDegrees(133.92)); // 5.54, 2.24, 360 - 146.61
  public static Pose2d end = start;

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull RamseteBuilder ramseteBuilder,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
    var toBall =
        TrajectoryGenerator.generateTrajectory(
            List.of(start, ball), trajConfig.get().setReversed(false));
    var fromBall =
        TrajectoryGenerator.generateTrajectory(
            List.of(ball, end), trajConfig.get().setReversed(true));
    if (field != null) field.getObject("ThreeBallAuto").setTrajectory(toBall.concatenate(fromBall));
    // assume that the robot is placed here at the start of auto
    return StationTwoBallAuto.createCommand(drive, cargo, ramseteBuilder, trajConfig, field)
        .andThen(new InstantCommand(cargo::runIntake))
        .andThen(ramseteBuilder.copy().traj(toBall).build())
        .andThen(new WaitCommand(1))
        .andThen(ramseteBuilder.copy().traj(fromBall).build())
        .andThen(new InstantCommand(cargo::spit))
        .andThen(new WaitCommand(1));
  }
}
