package frc.team449.oi.buttons;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.jetbrains.annotations.NotNull;

/** A version of {@link JoystickButton} that is a Button. */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
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
  @JsonCreator
  public SimpleButton(
      @NotNull @JsonProperty(required = true) final GenericHID joystick,
      @JsonProperty(required = true) final int buttonNumber) {
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
