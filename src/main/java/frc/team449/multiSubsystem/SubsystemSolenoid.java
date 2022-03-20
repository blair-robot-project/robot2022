package frc.team449.multiSubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.jetbrains.annotations.NotNull;

/** A subsystem with a single DoubleSolenoid piston. */
public interface SubsystemSolenoid {

  /** @param value The position to set the solenoid to. */
  void setSolenoid(@NotNull DoubleSolenoid.Value value);

  /** @return the current position of the solenoid. */
  @NotNull
  DoubleSolenoid.Value getSolenoidPosition();
}
