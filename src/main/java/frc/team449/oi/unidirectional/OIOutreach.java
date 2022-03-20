package frc.team449.oi.unidirectional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.team449.updatable.Updater;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class OIOutreach implements OIUnidirectional {

  /** The OI with higher priority that overrides if it has any input. */
  @NotNull private final OIUnidirectional overridingOI;

  /** The OI with lower priority that gets overriden. */
  @NotNull private final OIUnidirectional overridenOI;

  /** A button that overrides the lower priority controller */
  @NotNull private final Button button;

  /** The cached outputs for the left and right sides of the drive. */
  private @Nullable Pair<Double, Double> cachedLeftRightOutput;

  /** The cached forwards and rotational outputs. */
  private @Nullable Pair<Double, Double> cachedFwdRotOutput;

  /**
   * Default constructor
   *
   * @param overridingOI the controller for children's use
   * @param overridenOI the override controller with the full-stop button
   * @param stopButton the button to stop all robot functions while held
   */
  public OIOutreach(
      @NotNull final OIUnidirectional overridingOI,
      @NotNull final OIUnidirectional overridenOI,
      @NotNull final Button stopButton) {
    this.overridingOI = overridingOI;
    this.overridenOI = overridenOI;
    this.button = stopButton;
    Updater.subscribe(this);
  }

  /**
   * The output to be given to the left and right sides of the drive.
   *
   * @return A Pair of Doubles, where the 1st element is the output for the left and the second for
   *     the right, both from [-1, 1].
   */
  @Override
  public @NotNull Pair<Double, Double> getLeftRightOutput() {
    var leftRight = this.overridingOI.getLeftRightOutput();
    if (leftRight.getFirst() != 0 || leftRight.getSecond() != 0 || this.button.get()) {
      return this.overridingOI.getLeftRightOutput();
    } else {
      return this.overridenOI.getLeftRightOutput();
    }
  }

  /**
   * The cached output to be given to the left and right sides of the drive.
   *
   * @return A Pair of Doubles, where the 1st element is the output for the left and the second for
   *     the right, both from [-1, 1].
   */
  @Override
  @Log
  public @NotNull Pair<Double, Double> getLeftRightOutputCached() {
    if (this.cachedLeftRightOutput == null) {
      this.cachedLeftRightOutput = this.getLeftRightOutput();
    }
    return this.cachedLeftRightOutput;
  }

  /**
   * The forwards and rotational movement given to the drive.
   *
   * @return A Pair of Doubles, where the first element is the forwards output and the second is the
   *     rotational, both from [-1, 1]
   */
  @Override
  public @NotNull Pair<Double, Double> getFwdRotOutput() {
    var leftRight = this.overridingOI.getLeftRightOutput();
    if (leftRight.getFirst() != 0 || leftRight.getSecond() != 0 || this.button.get()) {
      return this.overridingOI.getFwdRotOutput();
    } else {
      return this.overridenOI.getFwdRotOutput();
    }
  }

  /**
   * The cached forwards and rotational movement given to the drive.
   *
   * @return A Pair of Doubles, where the first element is the forwards output and the second is the
   *     rotational, both from [-1, 1]
   */
  @Override
  @Log
  public @NotNull Pair<Double, Double> getFwdRotOutputCached() {
    if (this.cachedFwdRotOutput == null) {
      this.cachedFwdRotOutput = this.getFwdRotOutput();
    }
    return this.cachedFwdRotOutput;
  }

  /**
   * Whether the driver is trying to drive straight.
   *
   * @return True if the driver is trying to drive straight, false otherwise.
   */
  @Override
  @Log
  public boolean commandingStraight() {
    return this.getLeftRightOutputCached()
        .getFirst()
        .equals(this.getLeftRightOutputCached().getSecond());
  }

  /** Updates all cached values with current ones. */
  @Override
  public void update() {
    this.overridenOI.update();
    this.overridingOI.update();
    this.cachedLeftRightOutput = this.getLeftRightOutput();
    this.cachedFwdRotOutput = this.getFwdRotOutput();
  }
}
