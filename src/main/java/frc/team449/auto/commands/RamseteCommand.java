package frc.team449.auto.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.other.Clock;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class RamseteCommand extends CommandBase implements Loggable {
  @NotNull private final DriveUnidirectionalWithGyro drive;
  @NotNull private final RamseteController ramseteController;
  @NotNull private final Trajectory traj;
  @NotNull private final PIDController leftPid;
  @NotNull private final PIDController rightPid;
  @Log private double prevLeftVel;
  @Log private double prevRightVel;
  private double startTime;
  private double prevTime;

  public RamseteCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull RamseteController ramseteController,
      @NotNull Trajectory traj,
      @NotNull PIDController leftPid,
      @NotNull PIDController rightPid) {
    this.drive = drive;
    this.ramseteController = ramseteController;
    this.traj = traj;
    this.leftPid = leftPid;
    this.rightPid = rightPid;

    this.setName("Custom ramsete command");
  }

  @Override
  public void initialize() {
    this.startTime = Clock.currentTimeSeconds();
  }

  @Override
  public void execute() {
    var currTime = Clock.currentTimeSeconds();
    var dt = currTime - prevTime;

    var desiredState = traj.sample(currTime - startTime);
    var chassisSpeeds = ramseteController.calculate(drive.getCurrentPose(), desiredState);
    var wheelSpeeds = drive.getDriveKinematics().toWheelSpeeds(chassisSpeeds);

    var leftVel = wheelSpeeds.leftMetersPerSecond;
    var rightVel = wheelSpeeds.rightMetersPerSecond;

    var leftFF = drive.getFeedforward().calculate(leftVel, (leftVel - prevLeftVel) / dt);
    var rightFF = drive.getFeedforward().calculate(rightVel, (rightVel - prevRightVel) / dt);

    var leftFB = leftPid.calculate(drive.getLeftVel(), leftVel);
    var rightFB = rightPid.calculate(drive.getRightVel(), rightVel);

    this.prevLeftVel = leftVel;
    this.prevRightVel = rightVel;
    this.prevTime = currTime;

    drive.setVoltage(leftFF + leftFB, rightFF + rightFB);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      drive.setVoltage(0, 0);
    }
  }

  @Override
  public boolean isFinished() {
    return (Clock.currentTimeSeconds() - startTime) > traj.getTotalTimeSeconds();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("prevLeftVel", () -> prevLeftVel, x -> {});
    builder.addDoubleProperty("prevRightVel", () -> prevRightVel, x -> {});
  }
}
