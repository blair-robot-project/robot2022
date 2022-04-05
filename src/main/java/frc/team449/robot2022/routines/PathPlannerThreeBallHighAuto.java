package frc.team449.robot2022.routines;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team449.auto.commands.RamseteControllerUnidirectionalDrive;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.function.Supplier;

public class PathPlannerThreeBallHighAuto {

  public static Command createCommand(
          @NotNull DriveUnidirectionalWithGyro drive,
          @NotNull Cargo2022 cargo,
          @NotNull Supplier<TrajectoryConfig> trajConfig,
          Field2d field) {
    //    var traj = TrajectoryGenerator.generateTrajectory(List.of(start, pose1, pose2, end),
    // trajConfig.get());
    var traj =
            PathPlanner.loadPath(
                    "Curvy 5 ball", AutoConstants.AUTO_MAX_SPEED, AutoConstants.AUTO_MAX_ACCEL);
    field.getObject("Curvy 5 ball").setTrajectory(traj);
    return new RamseteControllerUnidirectionalDrive(drive, traj);
  }
}
