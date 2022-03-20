package frc.team449.drive.unidirectional.commands;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.drive.unidirectional.DriveUnidirectional;
import frc.team449.other.Clock;
import org.jetbrains.annotations.NotNull;

/** Go at a certain velocity for a set number of seconds */
public class DriveAtSpeed<T extends Subsystem & DriveUnidirectional> extends CommandBase {

  /** Speed to go at */
  private final double velocity;

  /** How long to run for */
  private final double seconds;

  /** The drive subsystem to execute this command on. */
  @NotNull private final T subsystem;

  /** When this command was initialized. */
  private long startTime;

  /**
   * Default constructor
   *
   * @param subsystem The drive to execute this command on
   * @param velocity How fast to go, in RPS
   * @param seconds How long to drive for.
   */
  public DriveAtSpeed(@NotNull T subsystem, double velocity, double seconds) {
    // Initialize stuff
    this.subsystem = subsystem;
    this.velocity = velocity;
    this.seconds = seconds;
    addRequirements(subsystem);
    Shuffleboard.addEventMarker(
        "Drive Robot bueno", this.getClass().getSimpleName(), EventImportance.kNormal);
    // Logger.addEvent("Drive Robot bueno", this.getClass());
  }

  /** Set up start time. */
  @Override
  public void initialize() {
    // Set up start time
    startTime = Clock.currentTimeMillis();
    // Reset drive velocity (for safety reasons)
    subsystem.fullStop();
    Shuffleboard.addEventMarker(
        "DriveAtSpeed init", this.getClass().getSimpleName(), EventImportance.kNormal);
    // Logger.addEvent("DriveAtSpeed init", this.getClass());
  }

  /** Send output to motors and log data */
  @Override
  public void execute() {
    // Set the velocity
    subsystem.setOutput(velocity, velocity);
  }

  /**
   * Exit after the command's been running for long enough
   *
   * @return True if timeout has been reached, false otherwise
   */
  @Override
  public boolean isFinished() {
    return (Clock.currentTimeMillis() - startTime) * 1e-3 > seconds;
  }

  /** Stop the drive when the command ends. */
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "DriveAtSpeed Interrupted! Stopping the robot.",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    // Brake on exit. Yes this should be setOutput because often we'll be testing how well the PID
    // loop handles a
    // full stop.
    subsystem.fullStop();
    Shuffleboard.addEventMarker(
        "DriveAtSpeed end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    // Logger.addEvent("DriveAtSpeed end.", this.getClass());
  }
}
