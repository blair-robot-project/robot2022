package frc.team449.oi.joystick;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.multiSubsystem.SubsystemWrapper;
import frc.team449.other.Util;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

/**
 * A command to rumble controllers given a supplier to provide rumble output. Runs until
 * interrupted.
 */
public class RumbleCommand extends CommandBase {
  /** The joysticks to rumble. */
  @NotNull private final List<GenericHID> joysticks;

  @NotNull private final RumbleComponent rumbleSupplier;

  /**
   * @param rumbleSupplier Give the rumble output for both sides of the joysticks.
   * @param joysticks The things to rumble.
   */
  public RumbleCommand(@NotNull RumbleComponent rumbleSupplier, @NotNull GenericHID... joysticks) {
    this.joysticks = Arrays.asList(joysticks);
    this.rumbleSupplier = rumbleSupplier;

    for (var joystick : joysticks) {
      addRequirements(new SubsystemWrapper<>(joystick));
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
