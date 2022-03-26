package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
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
   * Create an auto command that runs intake, goes to a ball, picks it up, comes back, and spits it
   * out. Despite the name, the robot may already have a preloaded ball xor may happen to pick up
   * two balls on the way
   *
   * @param toBallTraj The trajectory to go to the ball
   * @param fromBallTraj The trajectory to go back to the hub
   */
  public static Command getBallAndScore(
      @NotNull Cargo2022 cargo,
      @NotNull RamseteBuilder ramseteBuilder,
      @NotNull Trajectory toBallTraj,
      @NotNull Trajectory fromBallTraj,
      String name,
      @Nullable Field2d field) {
    var fullTraj = toBallTraj.concatenate(fromBallTraj);

    if (field != null) {
      field.getObject(name).setTrajectory(fullTraj);
    }

    // How much to wait to spit after the intake starts
    var spitWaitTime =
        fullTraj.getTotalTimeSeconds()
            - AutoConstants.PAUSE_BEFORE_INTAKE
            - AutoConstants.PAUSE_AFTER_SPIT;
    return ramseteBuilder
        .copy()
        .traj(fullTraj)
        .build()
        .alongWith(
            new WaitCommand(AutoConstants.PAUSE_BEFORE_INTAKE)
                .andThen(cargo::runIntake, cargo)
                .andThen(new WaitCommand(spitWaitTime))
                .andThen(cargo::spit, cargo));
  }

  /**
   * Create an auto command that runs intake, goes to a ball, picks it up, comes back, and spits it
   * out. Despite the name, the robot may already have a preloaded ball xor may happen to pick up
   * two balls on the way
   *
   * @param toBall The poses to hit on the way to the ball (including the start and end poses)
   * @param fromBall The poses to hit on the way back to the hub (including the start and end poses)
   */
  public static Command getBallAndScore(
      @NotNull Cargo2022 cargo,
      @NotNull RamseteBuilder ramseteBuilder,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      @NotNull List<Pose2d> toBall,
      @NotNull List<Pose2d> fromBall,
      String name,
      @Nullable Field2d field) {
    var toBallTraj =
        TrajectoryGenerator.generateTrajectory(toBall, trajConfig.get().setReversed(false));
    var fromBallTraj =
        TrajectoryGenerator.generateTrajectory(fromBall, trajConfig.get().setReversed(true));
    return AutoUtils.getBallAndScore(cargo, ramseteBuilder, toBallTraj, fromBallTraj, name, field);
  }

  /**
   * Create an auto command that runs intake, goes to a ball, picks it up, comes back, and spits it
   * out. Despite the name, the robot may already have a preloaded ball xor may happen to pick up
   * two balls on the way
   *
   * @param toBall The points to hit on the way to the ball (including the start and end poses)
   * @param fromBall The points to hit on the way back to the hub (including the start and end poses)
   */
  public static Command getBallAndScoreVectors(
      @NotNull Cargo2022 cargo,
      @NotNull RamseteBuilder ramseteBuilder,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      @NotNull List<Spline.ControlVector> toBall,
      @NotNull List<Spline.ControlVector> fromBall,
      String name,
      @Nullable Field2d field) {
    var toBallTraj =
        TrajectoryGenerator.generateTrajectory(
            new TrajectoryGenerator.ControlVectorList(toBall), trajConfig.get().setReversed(false));
    var fromBallTraj =
        TrajectoryGenerator.generateTrajectory(
            new TrajectoryGenerator.ControlVectorList(fromBall),
            trajConfig.get().setReversed(true));
    return AutoUtils.getBallAndScore(cargo, ramseteBuilder, toBallTraj, fromBallTraj, name, field);
  }
}
