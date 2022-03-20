package frc.team449.oi.unidirectional.arcade;

import frc.team449.oi.throttles.Throttle;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

/** A simple, two-stick arcade drive OI. */
public class OIArcadeSimple extends OIArcade {

  /** Left (rotation control) stick's throttle */
  @NotNull private final Throttle rotThrottle;

  /** Right (fwd/rev control) stick's throttle */
  @NotNull private final Throttle velThrottle;

  /**
   * Default constructor
   *
   * @param rotThrottle The throttle for rotating the robot.
   * @param velThrottle The throttle for driving straight.
   * @param rescaleOutputs Whether or not to scale the left and right outputs so the max output is
   *     1. Defaults to false.
   */
  public OIArcadeSimple(
      @NotNull final Throttle rotThrottle,
      @NotNull final Throttle velThrottle,
      final boolean rescaleOutputs) {
    super(rescaleOutputs);
    this.rotThrottle = rotThrottle;
    this.velThrottle = velThrottle;
  }

  /**
   * The forwards and rotational movement given to the drive.
   *
   * @return An array of length 2, where the first element is the forwards output and the second is
   *     the rotational, both from [-1, 1]
   */
  @Override
  @Log
  public double[] getFwdRotOutput() {
    return new double[] {velThrottle.getValue(), rotThrottle.getValue()};
  }
}
