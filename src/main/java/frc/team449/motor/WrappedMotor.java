package frc.team449.motor;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public final class WrappedMotor implements MotorController, Loggable {
  public final @NotNull Encoder encoder;
  private final @NotNull MotorController motor;
  /** Name for logging */
  private final @NotNull String name;
  @Log private double voltage;

  public WrappedMotor(
      @NotNull String name, @NotNull MotorController motor, @NotNull Encoder encoder) {
    this.motor = motor;
    this.encoder = encoder;
    this.name = name;
    MotorContainer.register(this);
  }

  @Override
  public void set(double speed) {
    motor.set(speed);
  }

  @Override
  public void setVoltage(double outputVolts) {
    motor.setVoltage(outputVolts);
    this.voltage = outputVolts;
  }

  @Log
  public double getVoltage(){
    return voltage;
  }

  @Log
  @Override
  public double get() {
    return motor.get();
  }

  public double getPositionUnits(){
    return encoder.getPositionUnits();
  }

  @Override
  public boolean getInverted() {
    return motor.getInverted();
  }

  @Override
  public void setInverted(boolean isInverted) {
    motor.setInverted(isInverted);
  }

  @Override
  public void disable() {
    motor.disable();
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }

  @Override
  public String configureLogName() {
    return this.name;
  }
}
