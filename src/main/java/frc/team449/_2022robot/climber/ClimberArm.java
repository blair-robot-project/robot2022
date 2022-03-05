package frc.team449._2022robot.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.team449.wrappers.WrappedMotor;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class ClimberArm extends ProfiledPIDSubsystem {
  private final WrappedMotor motor;
  private final ElevatorFeedforward feedforward;

  public ClimberArm(
      @NotNull WrappedMotor motor,
      @NotNull ProfiledPIDController controller,
      @NotNull ElevatorFeedforward feedforward) {
    super(controller);
    this.motor = motor;
    this.feedforward = feedforward;
  }

  @Override
  protected void useOutput(double output, TrapezoidProfile.State setpoint) {
    this.motor.setVoltage(output + feedforward.calculate(setpoint.velocity));
  }

  @Override
  protected double getMeasurement() {
    return this.motor.getPositionUnits();
  }

  @Log
  public double getError() {
    return this.getController().getPositionError();
  }

  public void stop() {
    this.getController().reset(0);
  }

  /**
   * Directly set the velocity. Only for testing/debugging
   */
  @Deprecated
  public void set(double velocity) {
    this.motor.set(velocity);
  }
}
