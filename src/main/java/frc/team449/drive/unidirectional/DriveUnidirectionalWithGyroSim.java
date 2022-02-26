package frc.team449.drive.unidirectional;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.team449.generalInterfaces.DriveSettings;
import frc.team449.generalInterfaces.updatable.Updatable;
import frc.team449.other.Clock;
import frc.team449.wrappers.AHRS;
import frc.team449.wrappers.WrappedMotor;
import org.jetbrains.annotations.NotNull;

public class DriveUnidirectionalWithGyroSim extends DriveUnidirectionalWithGyro implements Updatable {
  private final @NotNull DifferentialDrivetrainSim driveSim;
  private double lastTime;

  /**
   * Default constructor.
   *
   * @param leftMaster The master talon on the left side of the drive.
   * @param rightMaster The master talon on the right side of the drive.
   * @param ahrs The NavX gyro for calculating this drive's heading and angular velocity.
   * @param settings The settings for this drivetrain
   * @param driveSim The actual drivetrain simulation object
   */
  public DriveUnidirectionalWithGyroSim(
      @NotNull WrappedMotor leftMaster,
      @NotNull WrappedMotor rightMaster,
      @NotNull AHRS ahrs,
      @NotNull DriveSettings settings,
      @NotNull DifferentialDrivetrainSim driveSim) {
    super(leftMaster, rightMaster, ahrs, settings);

    this.driveSim = driveSim;
    this.lastTime = Clock.currentTimeSeconds();
  }

  @Override
  public void update() {
    var currTime = Clock.currentTimeSeconds();
    driveSim.update(currTime - this.lastTime);
    this.lastTime = currTime;
  }
}
