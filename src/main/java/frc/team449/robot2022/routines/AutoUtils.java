package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449.auto.builders.RamseteBuilder;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.List;
import java.util.function.Supplier;

/** Helpers for auto */
public final class AutoUtils {
  private AutoUtils() {}

  /**
   * Create an auto command that runs intake, goes to a ball, picks it up, comes back, and
   * spits it out. Despite the name, the robot may already have a preloaded ball, making it a
   * two-ball auto
   *
   * @param startPose The pose in which the robot starts
   * @param ballPose The pose at which the ball is found
   * @param endPose The pose to come back to and spit
   */
  public static Command oneBallAuto(
      @NotNull Cargo2022 cargo,
      @NotNull RamseteBuilder ramseteBuilder,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      @NotNull Pose2d startPose,
      @NotNull Pose2d ballPose,
      @NotNull Pose2d endPose,
      String name,
      @Nullable Field2d field) {
    var toBall =
        TrajectoryGenerator.generateTrajectory(
            List.of(startPose, ballPose), trajConfig.get().setReversed(false));
    var fromBall =
        TrajectoryGenerator.generateTrajectory(
            List.of(ballPose, endPose), trajConfig.get().setReversed(true));
    if (field != null) {
      field.getObject(name).setTrajectory(toBall.concatenate(fromBall));
    }
    return new InstantCommand(cargo::runIntake)
        .andThen(ramseteBuilder.copy().traj(toBall).build())
        .andThen(new WaitCommand(1))
        .andThen(ramseteBuilder.copy().traj(fromBall).build())
        .andThen(new InstantCommand(cargo::spit))
        .andThen(new WaitCommand(1));
  }
}
