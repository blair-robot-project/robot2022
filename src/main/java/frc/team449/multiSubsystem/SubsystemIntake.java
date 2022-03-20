package frc.team449.multiSubsystem;

import org.jetbrains.annotations.NotNull;

/** A subsystem used for intaking and possibly ejecting game pieces. */
public interface SubsystemIntake {

  /** @return the current mode of the intake. */
  @NotNull
  IntakeMode getMode();

  /** @param mode The mode to switch the intake to. */
  void setMode(@NotNull IntakeMode mode);

  /** An enum for the possible states of the intake. */
  enum IntakeMode {
    OFF,
    IN_SLOW,
    IN_FAST,
    OUT_SLOW,
    OUT_FAST
  }
}
