package frc.team449.drive.unidirectional;

import frc.team449.generalInterfaces.DriveSettings;
import frc.team449.wrappers.WrappedMotor;
import org.jetbrains.annotations.NotNull;

public class DriveUnidirectionalWithGyroSim extends DriveUnidirectionalBase {
  /**
   * @param leftMaster  The master on the left
   * @param rightMaster The master on the right
   * @param settings
   */
  public DriveUnidirectionalWithGyroSim(@NotNull WrappedMotor leftMaster, @NotNull WrappedMotor rightMaster, @NotNull DriveSettings settings) {
    super(leftMaster, rightMaster, settings);
  }
}
