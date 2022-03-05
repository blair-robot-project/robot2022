package frc.team449._2022robot.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.team449.generalInterfaces.updatable.Updatable;
import frc.team449.other.Updater;
import frc.team449.wrappers.WrappedMotor;
import org.jetbrains.annotations.NotNull;

public class ClimberArm extends PIDSubsystem {
  private final WrappedMotor motor;
  private final ElevatorFeedforward feedforward;

  public ClimberArm(
      @NotNull WrappedMotor motor,
      @NotNull PIDController controller,
      //todo feedforward isn't needed right now, so maybe only add later?
      @NotNull ElevatorFeedforward feedforward) {
    super(controller);
    this.motor = motor;
    this.feedforward = feedforward;
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    this.motor.setVoltage(output + feedforward.calculate(output));
  }

  @Override
  protected double getMeasurement() {
    return this.motor.getPositionUnits();
  }

  /**
   * Directly set the velocity. Only for testing/debugging
   */
  @Deprecated
  public void set(double velocity) {
    this.motor.set(velocity);
  }

  public static class ClimberArmSim extends ClimberArm implements Updatable {
    private final ElevatorSim sim;

    public ClimberArmSim(
        @NotNull WrappedMotor motor,
        @NotNull PIDController controller,
        @NotNull ElevatorFeedforward feedforward,
        double distanceTopBottom) {
      super(motor, controller, feedforward);
      this.sim = new ElevatorSim(DCMotor.getNeo550(1), 1.0, 1.0, 1.0, 0.0, distanceTopBottom);
      Updater.subscribe(this);
    }

    @Override
    public void update() {
      this.sim.setInput(RobotController.getBatteryVoltage());
    }
  }
}
