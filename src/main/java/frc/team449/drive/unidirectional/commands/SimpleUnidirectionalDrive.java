package frc.team449.drive.unidirectional.commands;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.drive.unidirectional.DriveUnidirectional;
import frc.team449.oi.unidirectional.OIUnidirectional;
import org.jetbrains.annotations.NotNull;

/** Very simple unidirectional drive control. */
public class SimpleUnidirectionalDrive<T extends Subsystem & DriveUnidirectional>
    extends CommandBase {

  /** The OI used for input. */
  @NotNull public final OIUnidirectional oi;

  /** The subsystem to execute this command on. */
  @NotNull private final T subsystem;

  /**
   * Default constructor
   *
   * @param subsystem The subsystem to execute this command on
   * @param oi The OI that gives the input to this command.
   */
  public SimpleUnidirectionalDrive(@NotNull T subsystem, @NotNull OIUnidirectional oi) {
    this.oi = oi;
    this.subsystem = subsystem;
    // Default commands need to require their subsystems.
    addRequirements(subsystem);
  }

  /** Stop the drive for safety reasons. */
  @Override
  public void initialize() {
    subsystem.fullStop();
  }

  /** Give output to the motors based on the stick inputs. */
  @Override
  public void execute() {
    subsystem.setOutput(oi.getLeftRightOutputCached()[0], oi.getLeftRightOutputCached()[1]);
  }

  /**
   * Run constantly because this is a default drive
   *
   * @return false
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Log and brake when interrupted. */
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "SimpleUnidirectionalDrive Interrupted! Stopping the robot.",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    // Brake for safety!
    subsystem.fullStop();
  }
}
