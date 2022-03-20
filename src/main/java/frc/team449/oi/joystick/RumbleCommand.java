package frc.team449.oi.joystick;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.other.Util;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A command to rumble controllers given a supplier to provide rumble output. Runs until
 * interrupted.
 */
public class RumbleCommand extends CommandBase {
  /**
   * Joysticks that have already been required by other RumbleCommands. Each joystick maps to a
   * dummy subsystem that's required by the RumbleCommand running on that joystick so that no
   * joystick has multiple RumbleCommands running on it at once
   */
  private static final Map<GenericHID, Subsystem> usedJoysticks = new HashMap<>();

  /** The joysticks to rumble. */
  @NotNull private final List<? extends GenericHID> joysticks;

  @NotNull private final RumbleComponent rumbleSupplier;

  /**
   * @param joysticks The things to rumble.
   * @param rumbleSupplier Give the rumble output for both sides of the joysticks.
   */
  public RumbleCommand(
      @NotNull List<? extends @NotNull GenericHID> joysticks,
      @NotNull RumbleComponent rumbleSupplier) {
    this.joysticks = joysticks;
    this.rumbleSupplier = rumbleSupplier;

    for (var joystick : joysticks) {
      if (!usedJoysticks.containsKey(joystick)) {
        usedJoysticks.put(joystick, new Subsystem() {});
      }
      // Tie the joystick to a dummy subsystem that represents the joystick
      addRequirements(usedJoysticks.get(joystick));
    }
  }

  @Override
  public void execute() {
    var leftRightRumble = rumbleSupplier.getOutput();
    this.rumbleJoysticks(
        Util.clamp(leftRightRumble.getFirst() / rumbleSupplier.maxOutput(), 0.0, 1.0),
        Util.clamp(leftRightRumble.getSecond() / rumbleSupplier.maxOutput(), 0.0, 1.0));
  }

  private void rumbleJoysticks(double leftRumble, double rightRumble) {
    for (var joystick : this.joysticks) {
      joystick.setRumble(GenericHID.RumbleType.kLeftRumble, leftRumble);
      joystick.setRumble(GenericHID.RumbleType.kRightRumble, rightRumble);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.rumbleJoysticks(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
