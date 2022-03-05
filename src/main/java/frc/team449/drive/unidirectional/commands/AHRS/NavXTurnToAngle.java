package frc.team449.drive.unidirectional.commands.AHRS;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.drive.unidirectional.DriveUnidirectional;
import frc.team449.generalInterfaces.ahrs.SubsystemAHRS;
import frc.team449.other.Clock;
import frc.team449.other.Util;
import org.jetbrains.annotations.NotNull;

/**
 * Turns to a specified angle, relative to the angle the AHRS was at when the robot was turned on.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class NavXTurnToAngle<T extends Subsystem & DriveUnidirectional & SubsystemAHRS>
    extends CommandBase {

  /** The drive subsystem to execute this command on and to get the gyro reading from. */
  @NotNull protected final T subsystem;

  /** The angle to turn to. */
  protected final double setpoint;

  /** How long this command is allowed to run for (in milliseconds) */
  private final long timeout;

  /** The time this command was initiated */
  protected long startTime;

  protected final PIDAngleController controller;

  /**
   * Default constructor.
   *
   * @param setpoint The setpoint, in degrees from 180 to -180.
   * @param timeout How long this command is allowed to run for, in seconds. Needed because
   *     sometimes floating-point errors prevent termination.
   * @param drive The drive subsystem to execute this command on.
   * @param controller The controller used to turn to the given setpoint
   */
  @JsonCreator
  public NavXTurnToAngle(
      double setpoint, double timeout, @NotNull T drive, @NotNull PIDAngleController controller) {
    this.subsystem = drive;
    this.setpoint = setpoint;
    // Convert from seconds to milliseconds
    this.timeout = (long) (timeout * 1000);
    this.controller = controller;
    addRequirements(subsystem);
  }

  /** Set up the start time and setpoint. */
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "NavXTurnToAngle init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    // Logger.addEvent("NavXTurnToAngle init.", this.getClass());
    // Set up start time
    this.startTime = Clock.currentTimeMillis();
    controller.setSetpoint(Util.clipTo180(setpoint));
  }

  /** Give output to the motors based on the output of the PID loop. */
  @Override
  public void execute() {
    // Process the output with deadband, minimum output, etc.
    double output = controller.getOutput(subsystem.getHeadingCached());

    // spin to the right angle
    subsystem.setOutput(-output, output);
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
    subsystem.fullStop();
    Shuffleboard.addEventMarker(
        "NavXTurnToAngle end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }
}
