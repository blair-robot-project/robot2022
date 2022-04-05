package frc.team449.drive.unidirectional.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.other.Clock;
import org.jetbrains.annotations.NotNull;

import java.util.function.DoubleSupplier;

/**
 * Turns to a specified angle, relative to the angle the AHRS was at when the robot was turned on.
 */
public class NavXTurnToAngle extends CommandBase {

  /** The drive subsystem to execute this command on and to get the gyro reading from. */
  @NotNull protected final DriveUnidirectionalWithGyro drive;

  /** Gives the angle to turn to */
  private final DoubleSupplier setpointSupplier;

  /** How long this command is allowed to run for (in milliseconds) */
  private final long timeout;

  /** The time this command was initiated */
  protected long startTime;

  protected final PIDAngleController controller;

  /**
   * Construct a {@link NavXTurnToAngle} command to turn to an (absolute) angle.
   *
   * @param setpoint The setpoint, in degrees from 180 to -180.
   * @param timeout How long this command is allowed to run for, in seconds. Needed because
   *     sometimes floating-point errors prevent termination.
   * @param drive The drive subsystem to execute this command on.
   * @param controller The controller used to turn to the given setpoint
   */
  public NavXTurnToAngle(
      double setpoint,
      double timeout,
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull PIDAngleController controller) {
    this(() -> setpoint, timeout, drive, controller);
  }

  /**
   * Construct a {@link NavXTurnToAngle} command that generates the setpoint only when the command
   * initializes, in case you want to turn to an angle relative to current heading
   *
   * @param setpointSupplier Supply the setpoint, in degrees from 180 to -180.
   * @param timeout How long this command is allowed to run for, in seconds. Needed because
   *     sometimes floating-point errors prevent termination.
   * @param drive The drive subsystem to execute this command on.
   * @param controller The controller used to turn to the given setpoint
   */
  public NavXTurnToAngle(
      DoubleSupplier setpointSupplier,
      double timeout,
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull PIDAngleController controller) {
    this.drive = drive;
    this.setpointSupplier = setpointSupplier;
    // Convert from seconds to milliseconds
    this.timeout = (long) (timeout * 1000);
    this.controller = controller;
    addRequirements(this.drive);
  }

  /**
   * Turn by a given angle from the heading at the start of the command
   *
   * @param setpoint The angle to turn by, in degrees
   * @param timeout How long this command is allowed to run for, in seconds. Needed because
   *     sometimes floating-point errors prevent termination.
   * @param drive The drive subsystem to execute this command on.
   * @param controller The controller used to turn to the given setpoint
   * @return A {@link NavXTurnToAngle} command that turns to the given relative angle (instead of an
   *     absolute angle)
   */
  public static NavXTurnToAngle createRelative(
      double setpoint,
      double timeout,
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull PIDAngleController controller) {
    return new NavXTurnToAngle(
        () -> drive.getAHRS().getCachedHeading() + setpoint, timeout, drive, controller);
  }

  /** Set up the start time and setpoint. */
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "NavXTurnToAngle init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    // Logger.addEvent("NavXTurnToAngle init.", this.getClass());
    // Set up start time
    this.startTime = Clock.currentTimeMillis();
    controller.setSetpoint(MathUtil.inputModulus(setpointSupplier.getAsDouble(), -180, 180));
  }

  /** Give output to the motors based on the output of the PID loop. */
  @Override
  public void execute() {
    // Process the output with deadband, minimum output, etc.
    double output = controller.getOutput(drive.getAHRS().getCachedHeading());

    // spin to the right angle
    drive.setOutput(-output, output);
  }

  /**
   * Exit when the robot reaches the setpoint or enough time has passed.
   *
   * @return True if timeout seconds have passed or the robot is on target, false otherwise.
   */
  @Override
  public boolean isFinished() {
    // The PIDController onTarget() is crap and sometimes never returns true because of floating
    // point errors, so
    // we need a timeout
    return controller.onTarget() || Clock.currentTimeMillis() - startTime > timeout;
  }

  /** Log when the command ends. */
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "NavXTurnToAngle interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
    drive.fullStop();
    Shuffleboard.addEventMarker(
        "NavXTurnToAngle end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }
}
