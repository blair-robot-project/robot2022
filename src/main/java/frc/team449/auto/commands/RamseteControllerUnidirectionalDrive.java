package frc.team449.auto.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.other.Util;
import org.jetbrains.annotations.NotNull;

import java.util.List;

public class RamseteControllerUnidirectionalDrive extends CommandBase {
  private final DriveUnidirectionalWithGyro drivetrain;
  private final RamseteController ramseteFeedback;
  private final PIDController leftController;
  private final PIDController rightController;
  private final Trajectory trajectory;
  private final SimpleMotorFeedforward rightFF;
  private final SimpleMotorFeedforward leftFF;
  private final Field2d field;
  private double startingTime;
  private double prevTime;

  /**
   * @param drivetrain
   * @param pidController PID controller to use (two copies will be made so both sides have the same
   *     PID)
   * @param trajectory Trajectory to follow
   * @param rightFF
   * @param leftFF
   * @param field
   */
  public RamseteControllerUnidirectionalDrive(
      @NotNull DriveUnidirectionalWithGyro drivetrain,
      @NotNull PIDController pidController,
      @NotNull Trajectory trajectory,
      @NotNull SimpleMotorFeedforward rightFF,
      @NotNull SimpleMotorFeedforward leftFF,
      Field2d field) {
    this.drivetrain = drivetrain;
    this.ramseteFeedback = new RamseteController();
    this.leftController = Util.copyPid(pidController);
    this.rightController = Util.copyPid(pidController);
    this.rightFF = rightFF;
    this.leftFF = leftFF;
    this.trajectory = trajectory;
    this.field = field;
    addRequirements(drivetrain);

    SmartDashboard.putData(
        "Ramsete PID",
        builder -> {
          builder.addDoubleProperty("left error", leftController::getPositionError, null);
          builder.addDoubleProperty("right error", rightController::getPositionError, null);
          builder.addDoubleProperty(
              "left vel",
              () -> leftController.getSetpoint() - leftController.getPositionError(),
              null);
          builder.addDoubleProperty(
              "right vel",
              () -> rightController.getSetpoint() - rightController.getPositionError(),
              null);
        });
  }

  @Override
  public void initialize() {
    if (field != null) {
      field.getObject("traj").setTrajectory(trajectory);
    }
    this.startingTime = Timer.getFPGATimestamp();
    this.prevTime = startingTime;
    leftController.reset();
    rightController.reset();
  }

  @Override
  public void execute() {
    var currTime = Timer.getFPGATimestamp();

    var currentWheelSpeeds = drivetrain.getWheelSpeeds();
    var targetWheelSpeeds =
        drivetrain
            .getDriveKinematics()
            .toWheelSpeeds(
                ramseteFeedback.calculate(
                    drivetrain.getCurrentPose(), trajectory.sample(currTime - startingTime)));

    double leftTarget = targetWheelSpeeds.leftMetersPerSecond;
    double rightTarget = targetWheelSpeeds.rightMetersPerSecond;
    double leftCurrent = currentWheelSpeeds.leftMetersPerSecond;
    double rightCurrent = currentWheelSpeeds.leftMetersPerSecond;

    double dt = currTime - prevTime;
    double leftFeedforward = leftFF.calculate(leftTarget, (leftTarget - leftCurrent) / dt);
    double rightFeedforward = rightFF.calculate(rightTarget, (rightTarget - rightCurrent) / dt);

    double leftOutput = leftFeedforward + leftController.calculate(leftCurrent, leftTarget);
    double rightOutput = rightFeedforward + rightController.calculate(rightCurrent, rightTarget);

    drivetrain.setVoltage(leftOutput, rightOutput);

    this.prevTime = currTime;
  }

  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startingTime) >= trajectory.getTotalTimeSeconds();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "Ramsete controller interrupted! Stopping the robot.",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    drivetrain.fullStop();
    Shuffleboard.addEventMarker(
        "Ramsete controller end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }

  @NotNull
  public static TrajectoryConfig basicTrajConfig(
      @NotNull DriveUnidirectionalWithGyro drive, double maxVel, double maxAccel) {
    var voltageConstraint =
        new DifferentialDriveVoltageConstraint(
            drive.getFeedforward(), drive.getDriveKinematics(), 12);
    return new TrajectoryConfig(maxVel, maxAccel)
        .setKinematics(drive.getDriveKinematics())
        .addConstraint(voltageConstraint);
  }
}
