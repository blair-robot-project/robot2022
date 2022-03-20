package frc.team449.drive.unidirectional.commands.AHRS;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.oi.unidirectional.tank.OITank;
import org.jetbrains.annotations.NotNull;

/** Drives straight using the NavX gyro to keep a constant alignment. */
public class NavXDriveStraight extends CommandBase {

  /** The drive subsystem to give output to. */
  @NotNull protected final DriveUnidirectionalWithGyro drive;

  /** The tank OI to get input from. */
  @NotNull private final OITank oi;

  @NotNull private final PIDAngleController controller;

  /**
   * Default constructor.
   *
   * @param drive The drive to execute this command on.
   * @param oi The tank OI to take input from.
   * @param controller Controller used to turn
   */
  public NavXDriveStraight(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull OITank oi,
      @NotNull PIDAngleController controller) {
    this.oi = oi;
    this.drive = drive;
    this.controller = controller;
    // This is likely to need to interrupt the DefaultCommand and therefore should require its
    // subsystem.
    addRequirements(drive);
  }

  /** Set the setpoint of the angle PID. */
  @Override
  public void initialize() {
    controller.setSetpoint(drive.getAHRS().getCachedHeading());
  }

  /** Give output to the drive based on the output of the PID loop. */
  @Override
  public void execute() {
    // Process the PID output with deadband, minimum output, etc.
    double output = controller.getOutput(drive.getAHRS().getCachedHeading());

    // Set throttle to the specified stick.
    drive.setOutput(
        oi.getLeftRightOutputCached()[0] - output, oi.getLeftRightOutputCached()[1] + output);
  }

  /**
   * Never finishes.
   *
   * @return false
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Log when this command ends */
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "NavXDriveStraight interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    drive.fullStop();
    Shuffleboard.addEventMarker(
        "NavXDriveStraight end", this.getClass().getSimpleName(), EventImportance.kNormal);
  }
}
