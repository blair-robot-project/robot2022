package frc.team449.robot2022.routines;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team449.auto.commands.RamseteControllerUnidirectionalDrive;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.function.Supplier;

/** Five ball auto that spits preloaded cargo, gets the */
public class CurvyFiveBallAuto {
  // Carry over the two ball auto and append to it another path that forms the three ball auto
  public static final Pose2d start = AutoUtils.pose(9.17, 5.86, 45.81);
  public static final Pose2d midBall = AutoUtils.pose(9.17, 5.86, 45.81);
  public static final Pose2d wallBall = AutoUtils.pose(9.17, 5.86, 45.81);

  public static final Pose2d end = start;

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
//    var firstBalls =
//        TrajectoryGenerator.generateTrajectory(
//            List.of(
//                    AutoUtils.pose(7.58, 2.97, -112.55)), trajConfig.get().setReversed(false));
//    var firstBack =
//        TrajectoryGenerator.generateTrajectory(List.of(), trajConfig.get().setReversed(true));
//    var lastBalls =
//        TrajectoryGenerator.generateTrajectory(List.of(), trajConfig.get().setReversed(false));
//    var lastBack =
//        TrajectoryGenerator.generateTrajectory(List.of(), trajConfig.get().setReversed(true));/
    var traj = PathPlanner.loadPath("Curvy 5 ball", AutoConstants.AUTO_MAX_SPEED, AutoConstants.AUTO_MAX_ACCEL);
    return new RamseteControllerUnidirectionalDrive(drive, traj);
  }
}
