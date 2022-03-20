package frc.team449.auto.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.AHRS.NavXTurnToAngle;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

/**
 * A backup command to be used only if Ramsete just does not work. Pretty stupid command, just goes
 * straight to each waypoint, then turns in place to face the next waypoint.
 */
public final class BraindeadAutoCommand {
  /**
   * @param drive
   * @param reversed Whether to go backwards
   * @param output Number in range [0, 1] (not [-1, 1]) representing portion of voltage to use for
   *     {@link DriveDistanceCommand}
   * @param pidAngleController
   * @param turnToAngleTimeout Timeout in seconds for the {@link NavXTurnToAngle} commands to turn
   *     in place.
   * @param startPose
   * @param endPose
   * @param waypoints
   * @return
   */
  @Contract("_, _, _, _, _, _, _, _ -> new")
  @NotNull
  public static Command create(
      @NotNull DriveUnidirectionalWithGyro drive,
      boolean reversed,
      double output,
      @NotNull PIDAngleController pidAngleController,
      double turnToAngleTimeout,
      @NotNull Pose2d startPose,
      @NotNull Pose2d endPose,
      @NotNull List<@NotNull Translation2d> waypoints) {
    var cmds = new ArrayList<Command>();
    cmds.add(new PrintCommand("BraindeadAutoCommand started"));
    cmds.add(new InstantCommand(() -> drive.resetOdometry(startPose)));

    var lastX = startPose.getTranslation().getX();
    var lastY = startPose.getTranslation().getY();

    // Make a new list (so we can mutate) and add the last pose to it to reduce code duplication
    waypoints = new ArrayList<>(waypoints);
    waypoints.add(endPose.getTranslation());

    var i = 0;
    for (var pos : waypoints) {
      var deltaX = pos.getX() - lastX;
      var deltaY = pos.getY() - lastY;

      var newAngle = Units.radiansToDegrees(Math.atan2(deltaY, deltaX));
      if (reversed) {
        newAngle += 180;
      }
      cmds.add(new PrintCommand("Turning to angle " + newAngle + ", " + i));
      cmds.add(new NavXTurnToAngle(newAngle, turnToAngleTimeout, drive, pidAngleController));
      cmds.add(new PrintCommand("Turned to angle " + newAngle + ", " + i));

      var dist = Math.hypot(deltaX, deltaY);
      cmds.add(new PrintCommand("Driving to distance " + dist + ", " + i));
      cmds.add(new DriveDistanceCommand(drive, dist, output, reversed));
      cmds.add(new PrintCommand("Drove to distance " + dist + ", " + i));

      lastX = pos.getX();
      lastY = pos.getY();

      i += 1;
    }

    cmds.add(
        new NavXTurnToAngle(
            endPose.getRotation().getDegrees(), turnToAngleTimeout, drive, pidAngleController));
    cmds.add(new PrintCommand("BraindeadAutoCommand over"));

    return new SequentialCommandGroup(cmds.toArray(Command[]::new));
  }
}
