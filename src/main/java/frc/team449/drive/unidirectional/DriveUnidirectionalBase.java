package frc.team449.drive.unidirectional;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.motor.WrappedMotor;
import frc.team449.other.Updater;
import io.github.oblarg.oblog.Loggable;
import org.jetbrains.annotations.NotNull;

public abstract class DriveUnidirectionalBase extends SubsystemBase
    implements DriveUnidirectional, Loggable {

  /** The master on the left */
  protected final @NotNull WrappedMotor leftMaster;
  /** The master on the right */
  protected final @NotNull WrappedMotor rightMaster;

  /** Drivetrain kinematics processor for measuring individual wheel speeds */
  @NotNull private final DifferentialDriveKinematics driveKinematics;

  // Cached values for various sensor readings
  private double cachedLeftVel = Double.NaN;
  private double cachedRightVel = Double.NaN;
  private double cachedLeftPos = Double.NaN;
  private double cachedRightPos = Double.NaN;

  /**
   * @param leftMaster The master on the left
   * @param rightMaster The master on the right
   */
  public DriveUnidirectionalBase(
      @NotNull WrappedMotor leftMaster,
      @NotNull WrappedMotor rightMaster,
      double trackWidth) {
    this.leftMaster = leftMaster;
    this.rightMaster = rightMaster;
    this.driveKinematics = new DifferentialDriveKinematics(trackWidth);
    Updater.subscribe(this);
  }

  @Override
  public void fullStop() {
    this.leftMaster.stopMotor();
    this.rightMaster.stopMotor();
  }

  /** Reset the position of the drive if it has encoders. */
  @Override
  public void resetPosition() {
    this.leftMaster.encoder.resetPosition();
    this.rightMaster.encoder.resetPosition();
  }

  /** Disable the motors. */
  public void disable() {
    this.leftMaster.disable();
    this.rightMaster.disable();
  }

  @Override
  public void enableMotors() {
    // todo can this method just be removed?
  }

  @Override
  public void setOutput(double left, double right) {
    this.setVoltage(
        left * RobotController.getBatteryVoltage(), right * RobotController.getBatteryVoltage());
  }

  /**
   * Set voltage output raw
   *
   * @param left The voltage output for the left side of the drive from [-12, 12]
   * @param right The voltage output for the right side of the drive from [-12, 12]
   */
  public void setVoltage(double left, double right) {
    this.leftMaster.setVoltage(left);
    this.rightMaster.setVoltage(right);
  }

  @Override
  public double getLeftVel() {
    return this.leftMaster.encoder.getVelocityUnits();
  }

  @Override
  public double getRightVel() {
    return this.rightMaster.encoder.getVelocityUnits();
  }

  @Override
  public double getLeftPos() {
    return this.leftMaster.encoder.getPositionUnits();
  }

  @Override
  public double getRightPos() {
    return this.rightMaster.encoder.getPositionUnits();
  }

  @NotNull
  @Override
  public Double getLeftVelCached() {
    return this.cachedLeftVel;
  }

  @NotNull
  @Override
  public Double getRightVelCached() {
    return this.cachedRightVel;
  }

  @Override
  public double getLeftPosCached() {
    return this.cachedLeftPos;
  }

  @NotNull
  @Override
  public Double getRightPosCached() {
    return this.cachedRightPos;
  }

  /**
   * @return Kinematics processor for wheel speeds
   */
  @NotNull
  public DifferentialDriveKinematics getDriveKinematics() {
    return this.driveKinematics;
  }

  /** Updates all cached values with current ones. */
  @Override
  public void update() {
    this.cachedLeftVel = this.getLeftVel();
    this.cachedLeftPos = this.getLeftPos();
    this.cachedRightVel = this.getRightVel();
    this.cachedRightPos = this.getRightPos();
  }
}
