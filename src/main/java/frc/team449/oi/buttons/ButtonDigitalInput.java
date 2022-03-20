package frc.team449.oi.buttons;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Button;
import org.jetbrains.annotations.NotNull;

/** A button triggered off of a digital input switch on the RoboRIO. */
public class ButtonDigitalInput extends Button {

  /** The input to read from. */
  @NotNull private final DigitalInput input;

  /**
   * Default constructor.
   *
   * @param input The input to read from.
   */
  public ButtonDigitalInput(@NotNull final DigitalInput input) {
    this.input = input;
  }

  /**
   * Get whether this button is pressed
   *
   * @return true if the all the ports in the MappedDigitalInput are true, false otherwise.
   */
  @Override
  public boolean get() {
    return this.input.get();
  }
}
