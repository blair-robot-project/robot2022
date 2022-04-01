package frc.team449.robot2022.routines;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449.auto.builders.RamseteBuilder;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.function.Supplier;

public class FiveBallAutoJames {


    public static final double FIRST_HALF_SPIT = 5.5;
    public static final double SECOND_HALF_SPIT = 5.5;
    public static final double SPIT_TIME = 0.5;


    public static Command createCommand(
            @NotNull DriveUnidirectionalWithGyro drive,
            @NotNull Cargo2022 cargo,
            @NotNull RamseteBuilder ramseteBuilder,
            Supplier<TrajectoryConfig> trajConfig,
            Field2d field) {

        Trajectory pathplanner_load = PathPlanner.loadPath("5 ball start", trajConfig.get().getMaxVelocity(), trajConfig.get().getMaxAcceleration());
        field.getObject("5ballbyjames").setTrajectory(pathplanner_load);

        return new InstantCommand(cargo::deployIntake, cargo)
                .andThen(
                        ramseteBuilder.copy().traj(pathplanner_load).build()
                                .alongWith(
                                        quickSpit(cargo, SPIT_TIME)
                                                .andThen(new WaitCommand(FIRST_HALF_SPIT))
                                                .andThen(quickSpit(cargo, SPIT_TIME))
                                                .andThen(new WaitCommand(SECOND_HALF_SPIT))
                                                .andThen(cargo::spit, cargo)
                                )
                );
    }

    //this call is blocking for any given command chain
    private static Command quickSpit(Cargo2022 cargo, double spitTime) {
        return new InstantCommand(cargo::spit, cargo)
                .andThen(new WaitCommand(spitTime))
                .andThen(cargo::runIntake, cargo);
    }

}