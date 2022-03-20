package frc.team449.robot2022.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.team449.motor.WrappedMotor;
import frc.team449.multiSubsystem.BooleanSupplierUpdatable;
import io.github.oblarg.oblog.Loggable;
import org.jetbrains.annotations.NotNull;

import java.util.function.BooleanSupplier;

public class ClimberArm extends ProfiledPIDSubsystem implements Loggable {
  private final @NotNull WrappedMotor motor;
  private final @NotNull ElevatorFeedforward feedforward;
  private final @NotNull BooleanSupplierUpdatable hallSensor;
  private final double midClimbLimit;

  private boolean reachedBottomCached;
  private boolean reachedMidLimitCached;

  /**
   * @param motor
   * @param controller
   * @param feedforward
   * @param midClimbLimit The height limit for mid climb. If the hall sensor is on and the arm is
   *     below half this height, the arm is considered to be at the bottom. If the hall sensor is on
   *     and the arm is above half this height, the arm is considered to be at the mid climb height
   *     limit.
   * @param hallSensor Hall effect sensor to determine if the climber is at either the bottom or mid
   *     climb height limit
   */
  public ClimberArm(
      @NotNull WrappedMotor motor,
      @NotNull ProfiledPIDController controller,
      @NotNull ElevatorFeedforward feedforward,
      double midClimbLimit,
      @NotNull BooleanSupplier hallSensor) {
    super(controller);
    this.motor = motor;
    this.feedforward = feedforward;
    this.midClimbLimit = midClimbLimit;
    this.hallSensor = new BooleanSupplierUpdatable(hallSensor, null);
  }

  /** Whether this arm's reached the bottom */
  public boolean reachedBottom() {
    return reachedBottomCached;
  }

  /** Whether this arm's reached the mid climb height limit */
  public boolean reachedMidLimit() {
    return reachedMidLimitCached;
  }

  @Override
  protected void useOutput(double output, TrapezoidProfile.State setpoint) {
    this.motor.setVoltage(output + feedforward.calculate(setpoint.velocity));
  }

  @Override
  public double getMeasurement() {
    return this.motor.getPositionUnits();
  }
  public void stop() {
    this.getController().reset(getMeasurement());
  }

  @Override
  public void periodic() {
    super.periodic();
    this.reachedBottomCached =
        this.hallSensor.getAsBoolean() && this.motor.getPositionUnits() < midClimbLimit / 2;
    this.reachedMidLimitCached =
        this.hallSensor.getAsBoolean() && this.motor.getPositionUnits() > midClimbLimit / 2;

    // Reset encoder position based on the Hall effect sensors
    if (reachedBottomCached) {
      motor.encoder.resetPosition(0);
    } else if (reachedMidLimitCached) {
      motor.encoder.resetPosition(this.midClimbLimit);
    }
  }

  /**
   * Only for testing/debugging
   */
  public void set(double velocity) {
    this.motor.set(velocity);
  }

  @Override
  public String configureLogName() {
    return "ClimberArm" + motor.configureLogName();
  }
}
