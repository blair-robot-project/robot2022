package frc.team449.oi.buttons;

import edu.wpi.first.wpilibj2.command.button.Button;
import org.jetbrains.annotations.NotNull;

/** Negates another {@link Button}. */
public class NegateButton extends Button {

  /** The button to negate. */
  @NotNull private final Button toNegate;

  /**
   * Default constructor.
   *
   * @param toNegate The button to negate.
   */
  public NegateButton(@NotNull final Button toNegate) {
    this.toNegate = toNegate;
  }

  /**
   * Get the opposite of toNegate's {@link Button#get()}.
   *
   * @return true if toNegate gets false, false otherwise.
   */
  @Override
  public boolean get() {
    return !this.toNegate.get();
  }
}
