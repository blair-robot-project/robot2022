package frc.team449.jacksonWrappers.simulated;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class DummyMotorController implements MotorController {
  @Override
  public void set(double speed) {}

  @Override
  public double get() {
    return 0;
  }

  @Override
  public boolean getInverted() {
    return false;
  }

  @Override
  public void setInverted(boolean isInverted) {}

  @Override
  public void disable() {}

  @Override
  public void stopMotor() {}
}
