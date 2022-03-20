package frc.team449.ahrs;

import org.jetbrains.annotations.NotNull;

/** A subsystem that has an AHRS on it. */
public interface SubsystemAHRS {
  @NotNull
  AHRS getAHRS();

  /** @return true if the gyroscope is currently overriden, false otherwise. */
  boolean getOverrideGyro();

  /** @param override true to override the gyro, false to un-override it. */
  void setOverrideGyro(boolean override);
}
