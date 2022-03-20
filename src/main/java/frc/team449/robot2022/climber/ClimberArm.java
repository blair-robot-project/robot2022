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

/**
 * A single arm of the 2022 climber. Periodically checks if the hall sensor is active. If it is,
 * then it determines whether the arm is at the bottom or mid climb height limit, and resets its
 * motor's encoder's position based on that
 */
public class ClimberArm extends ProfiledPIDSubsystem implements Loggable {
  private final @NotNull WrappedMotor motor;
  private final @NotNull ElevatorFeedforward feedforward;
  private final @NotNull BooleanSupplierUpdatable hallSensor;
  private final double midClimbLimit;

  /**
   * @param motor Winch motor used for retracting/extending arm
   * @param controller Profiled PID controller used for controlling this arm. Unused currently
   * @param feedforward Feedforward
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

  /** Whether this arm's reached the bottom. Based on encoder position */
  public boolean reachedBottom() {
    return motor.getPositionUnits() <= 0;
  }

  /** Whether this arm's reached the mid climb height limit. Based on encoder position */
  public boolean reachedMidLimit() {
    return motor.getPositionUnits() >= midClimbLimit;
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

    // Reset encoder position based on the Hall effect sensors
    if (this.hallSensor.getAsBoolean()) {
      if (motor.encoder.getPositionUnits() < midClimbLimit / 2) {
        // It's below half the mid climb height limit, so it's at the bottom
        motor.encoder.resetPosition(0);
      } else {
        // It's above half the mid climb height limit, so it's probably at the top
        motor.encoder.resetPosition(this.midClimbLimit);
      }
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
