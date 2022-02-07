package frc.team449.drive.unidirectional.commands.motionprofiling;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Collections;
import java.util.List;

/** Helper class to create commands that use WPI's {@link RamseteController} */
public class RamseteControllerCommands {

  public static Command goToPosition(
      @NotNull DriveUnidirectionalWithGyro drivetrain,
      double maxSpeedMeters,
      double maxAccelMeters,
      @Nullable Double maxCentripetalAcceleration,
      @NotNull PIDController leftPidController,
      @NotNull PIDController rightPidController,
      @NotNull Pose2d endingPose,
      @NotNull List<Translation2d> translations,
      boolean reversed) {
    // Create config for trajectory
    var config =
        new TrajectoryConfig(maxSpeedMeters, maxAccelMeters)
            .setKinematics(drivetrain.getDriveKinematics())
            .setReversed(reversed)
            .addConstraint(
                new DifferentialDriveVoltageConstraint(
                    drivetrain.getLeftFeedforwardCalculator(),
                    drivetrain.getDriveKinematics(),
                    12));

    if (maxCentripetalAcceleration != null) {
      config.addConstraint(new CentripetalAccelerationConstraint(maxCentripetalAcceleration));
    }

    var cmd = new RamseteCommand(
        TrajectoryGenerator.generateTrajectory(
            drivetrain.getCurrentPose(), translations, endingPose, config),
        drivetrain::getCurrentPose,
        new RamseteController(),
        drivetrain.getLeftFeedforwardCalculator(),
        drivetrain.getDriveKinematics(),
        drivetrain::getWheelSpeeds,
        leftPidController,
        rightPidController,
        drivetrain::setVoltage,
        drivetrain);
    SmartDashboard.putData("RamseteCommand", cmd);
    return cmd;
  }

  public static Command goToPointsWithDelay(
      @NotNull DriveUnidirectionalWithGyro drivetrain,
      double maxSpeedMeters,
      double maxAccelMeters,
      @Nullable Double maxCentripetalAcceleration,
      double waitSeconds,
      @NotNull PIDController leftPidController,
      @NotNull PIDController rightPidController,
      @NotNull List<Pose2d> poses,
      boolean reversed) {
    var cmd = new SequentialCommandGroup();
    for (var pose : poses) {
      cmd.addCommands(
          RamseteControllerCommands.goToPosition(
              drivetrain,
              maxSpeedMeters,
              maxAccelMeters,
              maxCentripetalAcceleration,
              leftPidController,
              rightPidController,
              pose,
              Collections.emptyList(),
              reversed),
          new WaitCommand(waitSeconds));
    }
    return cmd;
  }
}
