package frc.team449.robot2022.cargo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.motor.WrappedMotor;
import org.jetbrains.annotations.NotNull;

public final class FlywheelShooter extends SubsystemBase {
  private final WrappedMotor flywheelMotor;

  public FlywheelShooter(@NotNull WrappedMotor flywheelMotor) {
    this.flywheelMotor = flywheelMotor;
  }

  public void shootHigh() {
    flywheelMotor.set(CargoConstants.SHOOT_HIGH_OUTPUT);
  }

  public void shootLow() {
    flywheelMotor.set(CargoConstants.SHOOT_LOW_OUTPUT);
  }

  public void stop() {
    flywheelMotor.set(0);
  }
}
