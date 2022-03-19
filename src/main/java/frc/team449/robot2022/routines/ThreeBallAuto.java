package frc.team449.robot2022.routines;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449.auto.commands.RamseteControllerUnidirectionalDrive;
import frc.team449.components.TrajectoryGenerationQuinticComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.List;

public class ThreeBallAuto {
  // Carry over the two ball auto and append to it another path that forms the three ball auto
  public static double maxVel = StationTwoBallAuto.maxVel, maxAcc = StationTwoBallAuto.maxAcc;
  public static Pose2d start = StationTwoBallAuto.end;
  public static Pose2d ball =
      new Pose2d(5.41, 1.76, Rotation2d.fromDegrees(133.92)); // 5.54, 2.24, 360 - 146.61
  public static Pose2d end = start;

  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      double kP,
      double kD,
      @NotNull SimpleMotorFeedforward rightFF,
      @NotNull SimpleMotorFeedforward leftFF,
      Field2d field) {
    var config = RamseteControllerUnidirectionalDrive.basicTrajConfig(drive, maxVel, maxAcc);
    var toBall =
            TrajectoryGenerator.generateTrajectory(List.of(start, ball), config.setReversed(false));
    var fromBall =
            TrajectoryGenerator.generateTrajectory(List.of(ball, end), config.setReversed(true));
    if (field != null) field.getObject("ThreeBallAuto").setTrajectory(toBall.concatenate(fromBall));
    // assume that the robot is placed here at the start of auto
    return StationTwoBallAuto.createCommand(drive, cargo, kP, kD, rightFF, leftFF, field)
        .andThen(new InstantCommand(cargo::runIntake))
        .andThen(
            new RamseteControllerUnidirectionalDrive(
                drive, new PIDController(kP, 0, kD), toBall, rightFF, leftFF, field))
        .andThen(new WaitCommand(1))
        .andThen(
            new RamseteControllerUnidirectionalDrive(
                drive, new PIDController(kP, 0, kD), fromBall, rightFF, leftFF, field))
        .andThen(new InstantCommand(cargo::spit))
        .andThen(new WaitCommand(1));
  }
}
