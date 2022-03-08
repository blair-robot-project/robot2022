package frc.team449.drive.unidirectional;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.RobotController;
import org.jetbrains.annotations.NotNull;

public class DriveLoopBuilder {
  private LinearSystem<N2, N2, N2> drivePlant;
  private Vector<N2> errorTolerance;
  private double maxVolts = RobotController.getBatteryVoltage();
  private Vector<N2> stateStdDev;
  private Vector<N2> measStdDev;
  private double dtSeconds = 0.02;

  public DriveLoopBuilder drivePlant(@NotNull LinearSystem<N2, N2, N2> drivePlant) {
    this.drivePlant = drivePlant;
    return this;
  }

  public DriveLoopBuilder errorTolerance(double left, double right) {
    this.errorTolerance = VecBuilder.fill(left, right);
    return this;
  }

  public DriveLoopBuilder stateStdDev(double left, double right) {
    this.stateStdDev = VecBuilder.fill(left, right);
    return this;
  }

  public DriveLoopBuilder measStdDev(double left, double right) {
    this.measStdDev = VecBuilder.fill(left, right);
    return this;
  }

  /** Time between iterations of the control loop (20 milliseconds or 0.02 seconds) */
  public DriveLoopBuilder dtSeconds(double dtSeconds) {
    this.dtSeconds = dtSeconds;
    return this;
  }

  public DriveLoopBuilder maxVolts(double maxVolts) {
    this.maxVolts = maxVolts;
    return this;
  }

  public LinearSystemLoop<N2, N2, N2> build() {
    assert drivePlant != null;
    var controller =
        new LinearQuadraticRegulator<>(
            drivePlant, errorTolerance, VecBuilder.fill(maxVolts, maxVolts), dtSeconds);
    var observer =
        new KalmanFilter<>(Nat.N2(), Nat.N2(), drivePlant, stateStdDev, measStdDev, dtSeconds);
    var feedforward = new LinearPlantInversionFeedforward<>(drivePlant, dtSeconds);
    return new LinearSystemLoop<>(
        controller, feedforward, observer, RobotController.getBatteryVoltage());
  }
}
