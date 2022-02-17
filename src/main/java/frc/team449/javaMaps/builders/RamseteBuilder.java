package frc.team449.javaMaps.builders;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class RamseteBuilder {

  private @Nullable DriveUnidirectionalWithGyro drivetrain;
  private @Nullable PIDController leftPidController;
  private @Nullable PIDController rightPidController;
  private @Nullable Pose2d initialPose;
  private @Nullable Pose2d endingPose;
  private @Nullable List<Translation2d> translations;
  private final @NotNull List<TrajectoryConstraint> constraints = new ArrayList<>();
  private @Nullable Field2d field;
  private double maxSpeed;
  private double maxAccel;
  private boolean reversed;

  /** Set the drive subsystem which is to be controlled */
  public RamseteBuilder drivetrain(DriveUnidirectionalWithGyro drivetrain) {
    this.drivetrain = drivetrain;
    return this;
  }

  /** Set the PID controller for the left side. Uses velocity control. */
  public RamseteBuilder leftPidController(PIDController leftPidController) {
    this.leftPidController = leftPidController;
    return this;
  }

  /** Set the PID controller for the right side. Uses velocity control. */
  public RamseteBuilder rightPidController(PIDController rightPidController) {
    this.rightPidController = rightPidController;
    return this;
  }

  /** Set translations to perform on the way to the ending pose */
  public RamseteBuilder translations(Translation2d... translations) {
    this.translations = Arrays.asList(translations);
    return this;
  }

  /**
   * Add a constraint (e.g. a {@link
   * edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint} to constrain
   * centripetal acceleration)
   */
  public RamseteBuilder addConstraint(TrajectoryConstraint constraint) {
    this.constraints.add(constraint);
    return this;
  }

  /**
   * Set the {@link Field2d} widget in Glass to display the trajectory on (optional)
   */
  public RamseteBuilder field(Field2d field) {
    this.field = field;
    return this;
  }

  /**
   * Set max speed in m/s
   */
  public RamseteBuilder maxSpeed(double maxSpeed) {
    this.maxSpeed = maxSpeed;
    return this;
  }

  /**
   * Set max acceleration in m/s^2
   */
  public RamseteBuilder maxAccel(double maxAccel) {
    this.maxAccel = maxAccel;
    return this;
  }

  /**
   * Whether or not you're going backwards
   *
   * @param reversed {@code true} if going backwards, {@code false} if going forwards
   */
  public RamseteBuilder reversed(boolean reversed) {
    this.reversed = reversed;
    return this;
  }

  public Command build() {
    assert drivetrain != null : "Drivetrain must not be null";
    assert leftPidController != null : "Left PID controller must not be null";
    assert rightPidController != null : "Right PID controller must not be null";
    assert endingPose != null : "Ending pose must not be null";

    if (this.translations == null) {
      this.translations = List.of();
    }
    if (this.initialPose == null) {
      this.initialPose = drivetrain.getCurrentPose();
    }

    // Create config for trajectory
    var config =
        new TrajectoryConfig(maxSpeed, maxAccel)
            .setKinematics(drivetrain.getDriveKinematics())
            .setReversed(reversed)
            .addConstraint(
                new DifferentialDriveVoltageConstraint(
                    drivetrain.getLeftFeedforwardCalculator(),
                    drivetrain.getDriveKinematics(),
                    12));

    for (var constraint : this.constraints) {
      config.addConstraint(constraint);
    }

    // create trajectory from the current place where the robot is
    var traj =
        TrajectoryGenerator.generateTrajectory(this.initialPose, this.translations, this.endingPose, config);
    if (field != null) field.getObject("traj").setTrajectory(traj);

    var cmd =
        new RamseteCommand(
            traj,
            drivetrain::getCurrentPose,
            new RamseteController(),
            drivetrain.getLeftFeedforwardCalculator(),
            drivetrain.getDriveKinematics(),
            drivetrain::getWheelSpeeds,
            leftPidController,
            rightPidController,
            drivetrain::setVoltage,
            drivetrain);
    drivetrain.resetOdometry(initialPose);

    return cmd.andThen(() -> drivetrain.setVoltage(0, 0));
  }

  private RamseteBuilder() {}
}
