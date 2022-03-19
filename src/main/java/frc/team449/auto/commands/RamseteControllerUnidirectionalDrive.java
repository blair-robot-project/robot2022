package frc.team449.auto.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.components.TrajectoryGenerationComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.other.Util;
import org.jetbrains.annotations.NotNull;

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
  private double relativeTime;

  /**
   * @param drivetrain
   * @param pidController PID controller to use (two copies will be made so both sides have the same
   *     PID)
   * @param trajectoryGenerator
   * @param rightFF
   * @param leftFF
   * @param field
   */
  public RamseteControllerUnidirectionalDrive(
      @NotNull DriveUnidirectionalWithGyro drivetrain,
      @NotNull PIDController pidController,
      @NotNull TrajectoryGenerationComponent trajectoryGenerator,
      @NotNull SimpleMotorFeedforward rightFF,
      @NotNull SimpleMotorFeedforward leftFF,
      Field2d field) {
    this.drivetrain = drivetrain;
    ramseteFeedback = new RamseteController();
    leftController = Util.copyPid(pidController);
    rightController = Util.copyPid(pidController);
    this.rightFF = rightFF;
    this.leftFF = leftFF;
    trajectory = trajectoryGenerator.getTrajectory();
    this.field = field;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    if (field != null) {
      field.getObject("traj").setTrajectory(trajectory);
    }
    startingTime = Timer.getFPGATimestamp();
    relativeTime = startingTime;
    leftController.reset();
    rightController.reset();
  }

  @Override
  public void execute() {
    var currTime = Timer.getFPGATimestamp();
    var timeDelta = currTime - relativeTime;

    var currentWheelSpeeds = drivetrain.getWheelSpeeds();
    var targetWheelSpeeds =
        drivetrain
            .getDriveKinematics()
            .toWheelSpeeds(
                ramseteFeedback.calculate(
                    drivetrain.getCurrentPose(),
                    trajectory.sample(Timer.getFPGATimestamp() - startingTime)));
    var leftTarget = targetWheelSpeeds.leftMetersPerSecond;
    var rightTarget = targetWheelSpeeds.rightMetersPerSecond;

    var leftSpeedDelta =
        targetWheelSpeeds.leftMetersPerSecond - currentWheelSpeeds.leftMetersPerSecond;
    var rightSpeedDelta =
        targetWheelSpeeds.rightMetersPerSecond - currentWheelSpeeds.rightMetersPerSecond;

    var leftFeedforward = leftFF.calculate(leftTarget, leftSpeedDelta / timeDelta);
    var rightFeedforward = rightFF.calculate(rightTarget, rightSpeedDelta / timeDelta);

    drivetrain.setVoltage(
        leftFeedforward
            + leftController.calculate(currentWheelSpeeds.leftMetersPerSecond, leftTarget),
        rightFeedforward
            + rightController.calculate(currentWheelSpeeds.rightMetersPerSecond, rightTarget));

    relativeTime = currTime;
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
