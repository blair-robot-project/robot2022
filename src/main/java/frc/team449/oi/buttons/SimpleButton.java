package frc.team449.oi.buttons;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.jetbrains.annotations.NotNull;

/** A version of {@link JoystickButton} that is a Button. */
public class SimpleButton extends Button {

  /** The joystick the button is on. */
  @NotNull private final GenericHID joystick;

  /** The port of the button on the joystick. */
  private final int buttonNumber;

  /**
   * Default constructor.
   *
   * @param joystick The joystick the button is on.
   * @param buttonNumber The port of the button. Note that button numbers begin at 1, not 0.
   */
  public SimpleButton(@NotNull final GenericHID joystick, final int buttonNumber) {
    this.joystick = joystick;
    this.buttonNumber = buttonNumber;
  }

  /**
   * Get whether the button is pressed.
   *
   * @return true if the button is pressed, false otherwise.
   */
  @Override
  public boolean get() {
    return this.joystick.getRawButton(this.buttonNumber);
  }
}
