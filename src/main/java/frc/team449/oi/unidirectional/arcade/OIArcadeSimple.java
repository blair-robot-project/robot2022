package frc.team449.oi.unidirectional.arcade;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import frc.team449.generalInterfaces.doubleUnaryOperator.RampComponent;
import frc.team449.oi.throttles.Throttle;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

import java.util.function.DoubleUnaryOperator;

/** A simple, two-stick arcade drive OI. */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
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
  @JsonCreator
  public OIArcadeSimple(
      @NotNull Throttle rotThrottle,
      @NotNull Throttle velThrottle,
      @NotNull RampComponent fwdRamp,
      final boolean rescaleOutputs) {
    super(fwdRamp, rescaleOutputs);
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
  public double @NotNull [] getFwdRotOutputUnramped() {
    return new double[] {velThrottle.getValue(), rotThrottle.getValue()};
  }
}
