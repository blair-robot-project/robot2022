package frc.team449.motor.sim;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.jetbrains.annotations.NotNull;

public class SimMotor implements MotorController {
  @NotNull private final DCMotorSim motorSim;
  private double currOutput;
  private boolean isInverted = false;

  public SimMotor(@NotNull DCMotorSim motorSim) {
    this.motorSim = motorSim;
  }

  @Override
  public void set(double output) {
    this.currOutput = output;
    if (this.isInverted) {
      output *= -1;
    }
    motorSim.setInputVoltage(output * RobotController.getBatteryVoltage());
  }

  @Override
  public double get() {
    return this.currOutput;
  }

  @Override
  public void setInverted(boolean isInverted) {
    this.isInverted = isInverted;
  }

  @Override
  public boolean getInverted() {
    return this.isInverted;
  }

  @Override
  public void disable() {
    this.set(0);
  }

  @Override
  public void stopMotor() {
    this.set(0);
  }
}
