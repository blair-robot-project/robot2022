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

public class StationThreeBallHigh {

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      @NotNull Field2d field) {
    var traj =
        PathPlanner.loadPath(
            "ThreeStationHigh", AutoConstants.AUTO_MAX_SPEED, AutoConstants.AUTO_MAX_ACCEL);
    field.getObject("Three Station High").setTrajectory(traj);
    var totalTime = traj.getTotalTimeSeconds();
    var retractWaitTime = totalTime - 1.5;
    var shootWaitTime = totalTime - retractWaitTime - AutoConstants.SHOOT_HEADSTART;
    return new RamseteControllerUnidirectionalDrive(drive, traj)
        .alongWith(
            new InstantCommand(cargo::deployIntake, cargo)
                .andThen(cargo::runIntake, cargo)
                .andThen(cargo::deployHood, cargo)
                .andThen(new WaitCommand(1.6))
                .andThen(cargo::retractIntake, cargo)
                .andThen(cargo::stop, cargo)
                .andThen(new WaitCommand(1))
                .andThen(cargo::deployHood)
                .andThen(cargo.startShooterCommand())
                    .andThen(new WaitCommand(1.2))
                    .andThen(cargo::deployIntake, cargo)
                    .andThen(cargo::runIntake, cargo)
                    .andThen(new WaitCommand(1.8))
                    .andThen(cargo::retractIntake, cargo)
                    .andThen(cargo::stop, cargo)
                    .andThen(new WaitCommand(2.7))
                                    .andThen(cargo.startShooterCommand())
            ////                            .andThen(cargo::stop, cargo)
            //                    .andThen(cargo::deployHood, cargo)
            //                            .andThen(cargo::startShooterCommand, cargo)
            //                            .andThen(new WaitCommand(3))
            );
  }
}
