package frc.team449.auto.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import org.jetbrains.annotations.NotNull;

/** Simply drive a certain distance at some fraction of the voltage */
public class DriveDistanceCommand extends CommandBase {
  private final @NotNull DriveUnidirectionalWithGyro drive;
  private final double displacement;
  private final double output;
  private Translation2d startPos;

  /**
   * Create a command that makes the robot drive forward or backward a certain distance (in meters)
   * without any feedforward or feedback.
   *
   * @param drive The drivetrain to execute the command on
   * @param distance How far to travel (in meters)
   * @param output The motor output, in the range [0, 1] (not [-1, 1]). Automatically negated if
   *     {@code distance} is negative.
   * @param reversed Whether to drive backwards
   */
  public DriveDistanceCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      double distance,
      double output,
      boolean reversed) {
    this.drive = drive;
    this.displacement = reversed ? -distance : distance;
    this.output = reversed ? -output : output;
  }

  @Override
  public void initialize() {
    this.startPos = drive.getCurrentPose().getTranslation();
  }

  @Override
  public void execute() {
    drive.setOutput(output, output);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("drivedistancecommand ended");
    if (interrupted) {
      Shuffleboard.addEventMarker(
          this.getClass().getSimpleName(),
          this.getClass().getSimpleName() + " interrupted!",
          EventImportance.kLow);
    }
    drive.setVoltage(0, 0);
  }

  @Override
  public boolean isFinished() {
    var currPos = drive.getCurrentPose().getTranslation();
    var currDisplacement =
        Math.hypot(currPos.getY() - startPos.getY(), currPos.getX() - startPos.getX());
    return currDisplacement >= Math.abs(displacement);
  }
}
