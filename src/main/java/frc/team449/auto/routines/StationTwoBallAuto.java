package frc.team449.auto.routines;

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

public class StationTwoBallAuto {
  public static Pose2d start =
      new Pose2d(7.54, 1.87, Rotation2d.fromDegrees(180 + 89.32)); // TODO later
  public static Pose2d ball = new Pose2d(7.55, 1.06, Rotation2d.fromDegrees(180 + 90));
  public static Pose2d end = new Pose2d(7.77, 2.86, Rotation2d.fromDegrees(180 + 73));
  public static double maxVel = 2.5, maxAcc = .5;

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      double kP,
      double kD,
      @NotNull SimpleMotorFeedforward rightFF,
      @NotNull SimpleMotorFeedforward leftFF,
      Field2d field) {
    TrajectoryGenerationComponent traj1 =
        new TrajectoryGenerationQuinticComponent(
            drive, maxVel, maxAcc, List.of(start, ball), false);
    TrajectoryGenerationComponent traj2 =
        new TrajectoryGenerationQuinticComponent(drive, maxVel, maxAcc, List.of(ball, end), true);
    if (field != null)
      field
          .getObject("traj")
          .setTrajectory(traj1.getTrajectory().concatenate(traj2.getTrajectory()));
    // assume that the robot is placed here at the start of auto
    return new InstantCommand(cargo::deployIntake)
        .andThen(new InstantCommand(cargo::runIntake))
        .andThen(new InstantCommand(() -> drive.resetOdometry(start)))
        .andThen(
            new RamseteControllerUnidirectionalDrive(drive, kP, kD, traj1, rightFF, leftFF, field))
        .andThen(new WaitCommand(1))
        .andThen(
            new RamseteControllerUnidirectionalDrive(drive, kP, kD, traj2, rightFF, leftFF, field))
        .andThen(new InstantCommand(cargo::spit))
        .andThen(new WaitCommand(1));
  }
}
