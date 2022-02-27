package frc.team449.drive.unidirectional.commands.AHRS;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.drive.unidirectional.DriveUnidirectional;
import frc.team449.generalInterfaces.ahrs.SubsystemAHRS;
import frc.team449.other.Clock;
import org.jetbrains.annotations.NotNull;

/** Turn a certain number of degrees from the current heading. */
public class NavXTurnToAngleRelative<T extends Subsystem & DriveUnidirectional & SubsystemAHRS>
    extends NavXTurnToAngle<T> {

  /**
   * Default constructor.
   *
   * @param setpoint The setpoint, in degrees from 180 to -180.
   * @param timeout How long this command is allowed to run for, in seconds. Needed because
   *     sometimes floating-point errors prevent termination.
   * @param drive The drive subsystem to execute this command on.
   * @param controller The controller used to turn to the given setpoint
   */
  public NavXTurnToAngleRelative(
      double setpoint, double timeout, @NotNull T drive, @NotNull PIDAngleController controller) {
    super(setpoint, timeout, drive, controller);
  }

  /** Set up the start time and setpoint. */
  @Override
  public void initialize() {
    // Setup start time
    startTime = Clock.currentTimeMillis();
    Shuffleboard.addEventMarker(
        "NavXTurnToAngleRelative init.", getClass().getSimpleName(), EventImportance.kNormal);
    // Logger.addEvent("NavXRelativeTurnToAngle init.", this.getClass());
    // Do math to setup the setpoint.
    controller.setSetpoint(PIDAngleController.clipTo180(subsystem.getHeadingCached() + setpoint));
  }

  /** Log when the command ends. */
  @Override
  public void end(final boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "NavXTurnToAngleRelative interrupted!",
          getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    // how the heck do we stop this thing help

    Shuffleboard.addEventMarker(
        "NavXTurnToAngleRelative end.", getClass().getSimpleName(), EventImportance.kNormal);
  }
}
