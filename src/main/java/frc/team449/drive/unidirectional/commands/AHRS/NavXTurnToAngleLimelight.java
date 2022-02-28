package frc.team449.drive.unidirectional.commands.AHRS;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.components.limelight.LimelightDistanceComponent;
import frc.team449.drive.unidirectional.DriveUnidirectional;
import frc.team449.generalInterfaces.ahrs.SubsystemAHRS;
import frc.team449.generalInterfaces.limelight.Limelight;
import frc.team449.other.Clock;
import frc.team449.other.Util;
import org.jetbrains.annotations.NotNull;

/** Turn a certain number of degrees from the current heading, based on input from the limelight */
public class NavXTurnToAngleLimelight<T extends Subsystem & DriveUnidirectional & SubsystemAHRS>
    extends NavXTurnToAngleRelative<T> {

  private final Limelight limelight;

  /**
   * Default constructor.
   *
   * @param drive The drive subsystem to execute this command on.
   * @param timeout How long this command is allowed to run for, in seconds. Needed because
   *     sometimes floating-point errors prevent termination.
   */
  public NavXTurnToAngleLimelight(
      final double offset,
      @NotNull Limelight limelight,
      double timeout,
      @NotNull T drive,
      @NotNull PIDAngleController controller) {
    super(
        offset, // setpoint
        timeout,
        drive,
        controller);
    this.limelight = limelight;
  }

  /** Set up the start time and setpoint. */
  @Override
  public void initialize() {
    // Setup start time
    this.startTime = Clock.currentTimeMillis();
    Shuffleboard.addEventMarker(
        "NavXTurnToAngleLimelight init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    // Logger.addEvent("NavXRelativeTurnToAngle init.", this.getClass());
    // Do math to setup the setpoint.
    controller.setSetpoint(Util.clipTo180(subsystem.getHeadingCached() - limelight.getX()));
    // System.out.println("Current setpoint = " + limelight.getX());
    final LimelightDistanceComponent distanceComponent =
        new LimelightDistanceComponent(limelight, 20. / 12., 36, 7.5);
    System.out.println(distanceComponent.getAsDouble());
  }

  /** Log when the command ends. */
  @Override
  public void end(final boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "NavXTurnToAngleLimelight interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    // how the heck do we stop this thing help
    Shuffleboard.addEventMarker(
        "NavXRelativeTurnToAngleLimelight end.",
        this.getClass().getSimpleName(),
        EventImportance.kNormal);
  }
}
