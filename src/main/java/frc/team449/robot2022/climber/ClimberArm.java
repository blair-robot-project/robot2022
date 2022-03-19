package frc.team449.robot2022.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.team449.motor.WrappedMotor;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class ClimberArm extends ProfiledPIDSubsystem implements Loggable {
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
  public double getMeasurement() {
    return this.motor.getPositionUnits();
  }

  @Log
  public double getGoal() {
    return getController().getGoal().position;
  }

  @Log
  public double getSetpoint() {
    return getController().getSetpoint().position;
  }

  @Log
  public double getError() {
    return this.getController().getPositionError();
  }

  public void stop() {
    this.getController().reset(getMeasurement());
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

  public void sirswagger21() {
    System.out.println("Sirswagger21 my fav yt");
  }
}
