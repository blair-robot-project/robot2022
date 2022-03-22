package frc.team449.oi.buttons;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;
import org.jetbrains.annotations.NotNull;

/** A Button triggered by pushing the D-pad to a specific angle. */
public class DPadButton extends Button {

  /**
   * The angle that the D-pad must be pushed to to trigger this button. 0 degrees is probably
   * straight right.
   */
  private final int angle;

  /** The joystick with the relevant D-pad on it. */
  @NotNull private final GenericHID joystick;

  /**
   * Explicit argument constructor.
   *
   * @param joystick The joystick with the D-pad.
   * @param angle The angle that the D-pad must be pushed to to trigger this button.
   */
  public DPadButton(@NotNull final GenericHID joystick, final int angle) {
    this.angle = angle;
    this.joystick = joystick;
  }

  /**
   * Get whether this button is pressed
   *
   * @return true if the joystick's D-pad is pressed to the given angle, false otherwise.
   */
  @Override
  public boolean get() {
    return this.joystick.getPOV() == this.angle;
  }
}
