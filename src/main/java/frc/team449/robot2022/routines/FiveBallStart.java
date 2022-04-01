package frc.team449.robot2022.routines;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team449.auto.builders.RamseteBuilder;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.function.Supplier;

public class FiveBallStart {
  public static final Pose2d start = new Pose2d(9.17, 5.86, Rotation2d.fromDegrees(45.81));
  public static final Pose2d pose1 = new Pose2d(11.01, 6.22, Rotation2d.fromDegrees(12.99));
  public static final Pose2d pose2 = new Pose2d(11.41, 7.28, Rotation2d.fromDegrees(154.34));
  public static final Pose2d end = new Pose2d(9.59, 7.64, Rotation2d.fromDegrees(178.26));

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull RamseteBuilder ramseteBuilder,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
    //    var traj = TrajectoryGenerator.generateTrajectory(List.of(start, pose1, pose2, end),
    // trajConfig.get());
    var traj =
        PathPlanner.loadPath(
            "Curvy 5 ball", AutoConstants.AUTO_MAX_SPEED, .8);
    field.getObject("FiveBallStart").setTrajectory(traj);
    return ramseteBuilder.copy().traj(traj).build();
  }
}
