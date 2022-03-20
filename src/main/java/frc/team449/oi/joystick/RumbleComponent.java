package frc.team449.oi.joystick;

import edu.wpi.first.math.Pair;

public interface RumbleComponent {
  default void initialize() {}

  /** The max output of this RumbleComponent, to scale the output to [0, 1] */
  double maxOutput();

  /**
   * Get the rumble output for the left and right sides.
   *
   * @return A number in the range [0, {@link RumbleComponent#maxOutput() maxOutput}]
   */
  Pair<Double, Double> getOutput();
}
