package frc.team449.robot2022.routines;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.auto.commands.RamseteControllerUnidirectionalDrive;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.NavXTurnToAngle;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.function.Supplier;

public class StationFourBallHigh {

  public static Command createCommand(
          @NotNull DriveUnidirectionalWithGyro drive,
          @NotNull Cargo2022 cargo,
          @NotNull Supplier<TrajectoryConfig> trajConfig,
          Field2d field) {
    final double accel = 2.85;
    var dump1 =
            PathPlanner.loadPath(
                    "Station 4Ball High Pt1", AutoConstants.AUTO_MAX_SPEED, accel);
    field.getObject("Station 4Ball High Pt1").setTrajectory(dump1);
    var dump2 =
            PathPlanner.loadPath(
              "Station 4Ball High Pt2", AutoConstants.AUTO_MAX_SPEED, accel);
    field.getObject("Station 4Ball High Pt2").setTrajectory(dump2);
    var traj =
            PathPlanner.loadPath(
                    "Station 4Ball High", AutoConstants.AUTO_MAX_SPEED, accel);
    field.getObject("Station 4Ball High").setTrajectory(traj);
    return new RamseteControllerUnidirectionalDrive(drive, traj)
            .alongWith(
                    new InstantCommand(cargo::deployIntake)
                            .andThen(cargo::runIntake)
                            .andThen(cargo::deployHood)
                            .andThen(new WaitCommand(dump1.getTotalTimeSeconds() - 0.7))
                            .andThen(cargo::startShooterCommand)
                            .andThen(new WaitCommand(dump2.getTotalTimeSeconds() - 0.7))
                            .andThen(cargo::startShooterCommand)
            );
  }
}
