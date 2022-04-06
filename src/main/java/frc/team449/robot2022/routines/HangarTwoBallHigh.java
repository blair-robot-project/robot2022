package frc.team449.robot2022.routines;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
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
            Field2d field) {
        var traj =
                PathPlanner.loadPath(
                        "Hangar 2-Ball High", AutoConstants.AUTO_MAX_SPEED, .8);
        field.getObject("Hangar 2-Ball High").setTrajectory(traj);
        return new RamseteControllerUnidirectionalDrive(drive, traj);
    }
}
