package frc.team449.auto.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.drive.unidirectional.DriveFeedforward;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.other.Clock;
import org.jetbrains.annotations.NotNull;

/** Copied and slightly modified version of {@link edu.wpi.first.wpilibj2.command.RamseteCommand} */
public class RamseteCommand extends CommandBase {
  @NotNull private final DriveUnidirectionalWithGyro drive;
  @NotNull private final Trajectory traj;
  @NotNull private final RamseteController ramseteController;
  @NotNull private final DriveFeedforward feedforward;
  @NotNull private final PIDController leftPid;
  @NotNull private final PIDController rightPid;
  private DifferentialDriveWheelSpeeds prevSpeeds;
  private double startTime;
  private double prevTime;

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory. PID
   * control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
   * units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param drive The drivetrain to execute the command on
   * @param traj The trajectory to follow.
   * @param controller The RAMSETE controller used to follow the trajectory.
   * @param feedforward The feedforward to use for the drive.
   * @param leftPid The PIDController for the left side of the robot drive.
   * @param rightPid The PIDController for the right side of the robot drive.
   */
  public RamseteCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Trajectory traj,
      @NotNull RamseteController controller,
      @NotNull DriveFeedforward feedforward,
      @NotNull PIDController leftPid,
      @NotNull PIDController rightPid) {
    addRequirements(drive);

    this.drive = drive;
    this.traj = traj;
    this.ramseteController = controller;
    this.feedforward = feedforward;
    this.leftPid = leftPid;
    this.rightPid = rightPid;
  }

  @Override
  public void initialize() {
    this.startTime = Clock.currentTimeSeconds();
    this.prevTime = startTime;

    var initialState = traj.sample(0);
    this.prevSpeeds =
        drive
            .getDriveKinematics()
            .toWheelSpeeds(
                new ChassisSpeeds(
                    initialState.velocityMetersPerSecond,
                    0,
                    initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));

    leftPid.reset();
    rightPid.reset();
  }

  @Override
  public void execute() {
    double currTime = Clock.currentTimeSeconds();
    double dt = currTime - prevTime;

    var targetWheelSpeeds =
        drive
            .getDriveKinematics()
            .toWheelSpeeds(
                ramseteController.calculate(drive.getCurrentPose(), traj.sample(currTime)));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    var leftRightOutput =
        feedforward.calculate(
            leftSpeedSetpoint,
            rightSpeedSetpoint,
            (leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond) / dt,
            (rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond) / dt);

    double leftOutput = leftRightOutput.getFirst() + leftPid.calculate(drive.getLeftVel(), leftSpeedSetpoint);
    double rightOutput = leftRightOutput.getSecond() + rightPid.calculate(drive.getRightVel(), rightSpeedSetpoint);

    drive.setVoltage(leftOutput, rightOutput);
    feedforward.updateVoltages(leftOutput, rightOutput);

    prevSpeeds = targetWheelSpeeds;
    prevTime = currTime;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      drive.setVoltage(0.0, 0.0);
      feedforward.updateVoltages(0.0, 0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return Clock.currentTimeSeconds() - startTime > (traj.getTotalTimeSeconds());
  }
}
