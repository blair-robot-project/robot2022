package frc.team449.auto.routines;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449._2022robot.cargo.Cargo2022;
import frc.team449.auto.commands.RamseteControllerUnidirectionalDrive;
import frc.team449.components.TrajectoryGenerationComponent;
import frc.team449.components.TrajectoryGenerationQuinticComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import org.jetbrains.annotations.NotNull;

import java.util.List;

public class ThreeBallAuto {
    // Carry over the two ball auto and append to it another path that forms the three ball auto
    public static double maxVel = StationTwoBallAuto.maxVel, maxAcc = StationTwoBallAuto.maxAcc;
    public static Pose2d start = StationTwoBallAuto.end;
    public static Pose2d ball = new Pose2d(5.54, 2.24, Rotation2d.fromDegrees(360 - 146.61));
    public static Pose2d end = new Pose2d(7.8, 2.85, Rotation2d.fromDegrees(180 + 70));

    public static Command createCommand(@NotNull DriveUnidirectionalWithGyro drive,
                                        @NotNull Cargo2022 cargo,
                                        double kP,
                                        double kD,
                                        @NotNull SimpleMotorFeedforward rightFF,
                                        @NotNull SimpleMotorFeedforward leftFF,
                                        Field2d field
    ){
        TrajectoryGenerationComponent traj1 =
                new TrajectoryGenerationQuinticComponent(drive,
                        maxVel,
                        maxAcc,
                        List.of(start, ball),
                        false
                );
        TrajectoryGenerationComponent traj2 =
                new TrajectoryGenerationQuinticComponent(drive,
                        maxVel,
                        maxAcc,
                        List.of(ball, end),
                        true
                );
        if(field != null) field.getObject("traj").setTrajectory(traj1.getTrajectory().concatenate(traj2.getTrajectory()));
        // assume that the robot is placed here at the start of auto
        return StationTwoBallAuto.createCommand(drive, cargo, kP, kD, rightFF, leftFF, field)
                .andThen(new InstantCommand(cargo::runIntake))
                .andThen(new RamseteControllerUnidirectionalDrive(drive, new PIDController(kP, 0, kD), traj1, rightFF, leftFF, field))
                .andThen(new WaitCommand(1))
                .andThen(new RamseteControllerUnidirectionalDrive(drive, new PIDController(kP, 0, kD), traj2, rightFF, leftFF, field))
                .andThen(new InstantCommand(cargo::spit))
                .andThen(new WaitCommand(1));
    }
}
