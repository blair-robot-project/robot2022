package frc.team449.multiSubsystem;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.RobotController;
import org.jetbrains.annotations.NotNull;

/**
 * Builds a {@link LinearSystemLoop} according to <a
 * href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-flywheel-walkthrough.html">this
 * tutorial</a>
 *
 * @param <N> The dimension of the state
 */
public final class StateSpaceModelBuilder<N extends Num & Nat<N>> {
  private LinearSystem<N, N, N> plant;
  private double loopTime = 0.02;
  private final Nat<N> dim;
  private Vector<N> stateStdDevs;
  private Vector<N> measStdDevs;
  private Vector<N> errorTolerances;
  private Vector<N> maxControlEfforts;

  /**
   * @param dim A concrete instance of {@link N}, the number of dimensions. Why? Because Java's
   *     generics suck
   */
  public StateSpaceModelBuilder(@NotNull Nat<N> dim) {
    this.dim = dim;
  }

  public StateSpaceModelBuilder<N> plant(@NotNull LinearSystem<N, N, N> plant) {
    this.plant = plant;
    return this;
  }

  /** How many seconds the control loop takes (0.02 by default) */
  public StateSpaceModelBuilder<N> loopTime(double seconds) {
    this.loopTime = seconds;
    return this;
  }

  /**
   * Standard deviation of the model, i.e., how much you trust the model. Used for the KalmanFilter
   */
  public StateSpaceModelBuilder<N> stateStdDevs(@NotNull Vector<N> stateStdDevs) {
    this.stateStdDevs = stateStdDevs;
    return this;
  }

  /**
   * Standard deviation of encoder readings (position, velocity, whatever), i.e., how much you trust
   * the encoders. Used for the KalmanFilter
   */
  public StateSpaceModelBuilder<N> measStdDevs(@NotNull Vector<N> measStdDevs) {
    this.measStdDevs = measStdDevs;
    return this;
  }

  /** Error tolerance for velocity or whatever states you have */
  public StateSpaceModelBuilder<N> errorTolerances(@NotNull Vector<N> tolerances) {
    this.errorTolerances = tolerances;
    return this;
  }

  /**
   * Max control effort (voltage). According to WPI docs, 12, the approximate max battery voltage,
   * is a good starting point, but you can decrease it to make the controller less aggressive
   */
  public StateSpaceModelBuilder<N> maxControlEfforts(@NotNull Vector<N> controlEfforts) {
    this.maxControlEfforts = controlEfforts;
    return this;
  }

  /** Build a {@link LinearSystemLoop} with the properties given above. */
  @NotNull
  public LinearSystemLoop<N, N, N> build() {
    var observer = new KalmanFilter<>(dim, dim, plant, stateStdDevs, measStdDevs, loopTime);
    var controller =
        new LinearQuadraticRegulator<>(plant, errorTolerances, maxControlEfforts, loopTime);
    return new LinearSystemLoop<>(
        plant, controller, observer, RobotController.getBatteryVoltage(), loopTime);
  }
}
