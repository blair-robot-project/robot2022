package frc.team449.robot2022.routines;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449.auto.commands.RamseteControllerUnidirectionalDrive;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.function.Supplier;

public class HangarTwoBallHigh {

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      @NotNull Field2d field) {
    var traj =
        PathPlanner.loadPath(
            "Hangar 2-Ball High", AutoConstants.AUTO_MAX_SPEED, AutoConstants.AUTO_MAX_ACCEL);
    field.getObject("Hangar 2-Ball High").setTrajectory(traj);
    return new RamseteControllerUnidirectionalDrive(drive, traj)
        .alongWith(
            new InstantCommand(cargo::deployIntake, cargo)
                .andThen(cargo::runIntake, cargo)
                .andThen(cargo::removeHood, cargo)
                .andThen(new WaitCommand(traj.getTotalTimeSeconds() - 1.5))
                .andThen(cargo::retractIntake, cargo)
                .andThen(cargo::stop, cargo)
                .andThen(cargo::startShooterCommand, cargo));
  }
}
