package frc.team449.auto.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class RamseteControllerUnidirectionalDrive extends CommandBase implements Loggable {
  private final DriveUnidirectionalWithGyro drivetrain;
  private final RamseteController ramseteFeedback;
  private final PIDController leftController;
  private final PIDController rightController;
  private final Trajectory trajectory;
  private final SimpleMotorFeedforward feedforward;
  private double startingTime;
  private double prevTime;
  private DifferentialDriveWheelSpeeds previousSpeeds;
  @Log private double desiredLeftVoltage;
  @Log private double desiredRightVoltage;
  /**
   * @param drivetrain
   * @param leftController Velocity PID controller to use for the left side
   * @param rightController Velocity PID controller to use for the right side
   * @param trajectory Trajectory to follow
   * @param feedforward
   */
  public RamseteControllerUnidirectionalDrive(
      @NotNull DriveUnidirectionalWithGyro drivetrain,
      @NotNull PIDController leftController,
      @NotNull PIDController rightController,
      @NotNull Trajectory trajectory,
      @NotNull SimpleMotorFeedforward feedforward) {
    this.drivetrain = drivetrain;
    this.ramseteFeedback = new RamseteController();
    this.leftController = leftController;
    this.rightController = rightController;
    this.feedforward = feedforward;
    this.trajectory = trajectory;
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
          builder.addDoubleProperty("left desired", () -> desiredLeftVoltage, x -> {});
          builder.addDoubleProperty("right desired", () -> desiredRightVoltage, x -> {});
        });
  }

  @Override
  public void initialize() {
    this.startingTime = Timer.getFPGATimestamp();
    this.prevTime = startingTime;
    var initialState = trajectory.sample(0);
    previousSpeeds =
        drivetrain
            .getDriveKinematics()
            .toWheelSpeeds(
                new ChassisSpeeds(
                    initialState.velocityMetersPerSecond,
                    0,
                    initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
    leftController.reset();
    rightController.reset();
    drivetrain.resetOdometry(trajectory.getInitialPose());
    ramseteFeedback.setEnabled(false);
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

    double leftDelta = targetWheelSpeeds.leftMetersPerSecond - previousSpeeds.leftMetersPerSecond;
    double rightDelta =
        targetWheelSpeeds.rightMetersPerSecond - previousSpeeds.rightMetersPerSecond;
    double dt = currTime - prevTime;
    double leftFeedforward = feedforward.calculate(leftTarget, leftDelta / dt);
    double rightFeedforward = feedforward.calculate(rightTarget, rightDelta / dt);

    this.desiredLeftVoltage = leftFeedforward + leftController.calculate(leftCurrent, leftTarget);
    this.desiredRightVoltage =
        rightFeedforward + rightController.calculate(rightCurrent, rightTarget);

    drivetrain.setVoltage(desiredLeftVoltage, desiredRightVoltage);

    previousSpeeds = targetWheelSpeeds;
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
}
