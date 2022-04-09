package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.auto.commands.RamseteControllerUnidirectionalDrive;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.NavXTurnToAngle;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.function.Supplier;

public class HangarTwoBallHighStraight {
    public static final Pose2d start = AutoUtils.pose(6.12, 5.07, 133.15);
    public static final Pose2d ball = AutoUtils.pose(5.27, 5.88, 130.16);
    public static final Pose2d end = AutoUtils.pose(7.07, 4.79, -21.04);

    public static Command createCommand(
            @NotNull DriveUnidirectionalWithGyro drive,
            @NotNull Cargo2022 cargo,
            @NotNull Supplier<PIDAngleController> angleController,
            @NotNull Supplier<TrajectoryConfig> trajConfig,
            Field2d field) {
        var ballTraj =
                TrajectoryGenerator.generateTrajectory(
                        List.of(start, ball), trajConfig.get().setReversed(false));
        var hubTraj =
                TrajectoryGenerator.generateTrajectory(
                        List.of(ball, end), trajConfig.get().setReversed(false));
        field.getObject("ThreeBallHighStraightReverse").setTrajectory(ballTraj);
        return AutoUtils.getBallAndScoreHigh(
                drive,
                cargo,
                angleController.get(),
                trajConfig,
                List.of(start, ball),
                List.of(ball, end),
                HangarTwoBallHighStraight.class.getSimpleName(),
                field,
                true);
//
//                new InstantCommand(cargo::deployIntake, cargo)
//                .andThen(cargo::deployHood, cargo)
//                .andThen(cargo::runIntake, cargo)
//                .andThen(new RamseteControllerUnidirectionalDrive(drive, ballTraj))
//                .andThen(NavXTurnToAngle.createRelative(180, AutoConstants.TURN_TIMEOUT, drive, angleController.get()))
//                .andThen(new RamseteControllerUnidirectionalDrive(drive, hubTraj))
//                .andThen(cargo::startShooterCommand, cargo);
    }
}
