package frc.team449.oi.unidirectional;

import edu.wpi.first.math.Pair;
import frc.team449.updatable.Updatable;
import io.github.oblarg.oblog.Loggable;
import org.jetbrains.annotations.NotNull;

/**
 * An OI to control a robot with a unidirectional drive that has a left and right side (e.g. not
 * meccanum, swerve, or holonomic)
 */
public interface OIUnidirectional extends Updatable, Loggable {

  /**
   * The output to be given to the left and right sides of the drive.
   *
   * @return A Pair of Doubles, where the 1st element is the output for the left and the second
   *     for the right, both from [-1, 1].
   */
  @NotNull Pair<Double, Double> getLeftRightOutput();

  /**
   * The cached output to be given to the left and right sides of the drive.
   *
   * @return A Pair of Doubles, where the 1st element is the output for the left and the second
   *     for the right, both from [-1, 1].
   */
  @NotNull Pair<Double, Double> getLeftRightOutputCached();

  /**
   * The cached output to be given to the left side of the drive, in the range [-1, 1].
   */
  default double getLeftOutputCached() {
    return this.getLeftRightOutputCached().getFirst();
  }

  /**
   * The cached output to be given to the right side of the drive, in the range [-1, 1].
   */
  default double getRightOutputCached() {
    return this.getLeftRightOutputCached().getSecond();
  }

  /**
   * The forwards and rotational movement given to the drive.
   *
   * @return A Pair of Doubles, where the first element is the forwards output and the second is
   *     the rotational, both from [-1, 1]
   */
  @NotNull Pair<Double, Double> getFwdRotOutput();

  /**
   * The cached forwards and rotational movement given to the drive.
   *
   * @return A Pair of Doubles, where the first element is the forwards output and the second is
   *     the rotational, both from [-1, 1]
   */
  @NotNull Pair<Double, Double> getFwdRotOutputCached();

  /**
   * The cached forwards movement given to the drive.
   *
   * @return The forwards output, from [-1, 1]
   */
  default double getFwdOutputCached() {
    return getFwdRotOutputCached().getFirst();
  }

  /**
   * The cached rotational movement given to the drive.
   *
   * @return The forwards output, from [-1, 1]
   */
  default double getRotOutputCached() {
    return getFwdRotOutputCached().getSecond();
  }

  /**
   * Whether the driver is trying to drive straight.
   *
   * @return True if the driver is trying to drive straight, false otherwise.
   */
  boolean commandingStraight();
}
