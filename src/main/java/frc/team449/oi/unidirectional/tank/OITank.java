package frc.team449.oi.unidirectional.tank;

import edu.wpi.first.math.Pair;
import frc.team449.oi.unidirectional.OIUnidirectional;
import frc.team449.updatable.Updater;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/** A tank-style dual joystick OI. */
public abstract class OITank implements OIUnidirectional {

  /** Cached left and right output. */
  private @Nullable Pair<Double, Double> leftRightOutputCached;

  /** Cached forwards and rotational output. */
  private @Nullable Pair<Double, Double> fwdRotOutputCached;

  protected OITank() {
    Updater.subscribe(this);
  }

  /**
   * Get the throttle for the left side of the drive.
   *
   * @return percent of max speed for left motor cluster from [-1.0, 1.0]
   */
  @Log
  public abstract double getLeftThrottle();

  /**
   * Get the throttle for the right side of the drive.
   *
   * @return percent of max speed for right motor cluster from [-1.0, 1.0]
   */
  @Log
  public abstract double getRightThrottle();

  /**
   * The output to be given to the left and right sides of the drive.
   *
   * @return A Pair of Doubles, where the 1st element is the output for the left and the second
   *     for the right, both from [-1, 1].
   */
  @Override
  @Log
  public @NotNull Pair<Double, Double> getLeftRightOutput() {
    return Pair.of(getLeftThrottle(), getRightThrottle());
  }

  /**
   * The cached output to be given to the left and right sides of the drive.
   *
   * @return A Pair of Doubles, where the 1st element is the output for the left and the second
   *     for the right, both from [-1, 1].
   */
  @Override
  @Log
  public @NotNull Pair<Double, Double> getLeftRightOutputCached() {
    if (leftRightOutputCached == null) {
      this.leftRightOutputCached = getLeftRightOutput();
    }
    return leftRightOutputCached;
  }

  /**
   * The forwards and rotational movement given to the drive.
   *
   * @return A Pair of Doubles, where the first element is the forwards output and the second is
   *     the rotational, both from [-1, 1]
   */
  @Override
  @Log
  public @NotNull Pair<Double, Double> getFwdRotOutput() {
    return Pair.of(
        (getLeftThrottle() + getRightThrottle()) / 2.,
        (getLeftThrottle() - getRightThrottle()) / 2.);
  }

  /**
   * The cached forwards and rotational movement given to the drive.
   *
   * @return A Pair of Doubles, where the first element is the forwards output and the second is
   *     the rotational, both from [-1, 1]
   */
  @Override
  @Log
  public @NotNull Pair<Double, Double> getFwdRotOutputCached() {
    if (fwdRotOutputCached == null) {
      this.fwdRotOutputCached = this.getFwdRotOutput();
    }
    return fwdRotOutputCached;
  }

  /** Updates all cached values with current ones. */
  @Override
  public void update() {
    leftRightOutputCached = getLeftRightOutput();
    fwdRotOutputCached = getFwdRotOutput();
  }
}
