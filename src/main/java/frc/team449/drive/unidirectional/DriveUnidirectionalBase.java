package frc.team449.drive.unidirectional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.drive.DriveSettings;
import frc.team449.motor.WrappedMotor;
import io.github.oblarg.oblog.Loggable;
import org.jetbrains.annotations.NotNull;

public class DriveUnidirectionalBase extends SubsystemBase
    implements DriveUnidirectional, Loggable {

  /** The master on the left */
  protected final @NotNull WrappedMotor leftMaster;
  /** The master on the right */
  protected final @NotNull WrappedMotor rightMaster;

  /** Current settings for the drivetrain */
  protected final @NotNull DriveSettings settings;

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
      @NotNull DriveSettings settings) {
    this.leftMaster = leftMaster;
    this.rightMaster = rightMaster;
    this.settings = settings;
  }

  @Override
  public void fullStop() {
    this.leftMaster.stopMotor();
    this.rightMaster.stopMotor();
  }

  /** Reset the position of the drive if it has encoders. */
  @Override
  public void resetEncoders() {
    this.leftMaster.encoder.resetPosition(0);
    this.rightMaster.encoder.resetPosition(0);
  }

  /** Disable the motors. */
  public void disable() {
    this.leftMaster.disable();
    this.rightMaster.disable();
  }

  @Override
  public void setOutput(double left, double right) {
    this.leftMaster.set(left);
    this.rightMaster.set(right);
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

  @Override
  public double getLeftVelCached() {
    if (Double.isNaN(this.cachedLeftVel)) {
      this.cachedLeftVel = this.getLeftVel();
    }
    return this.cachedLeftVel;
  }

  @Override
  public double getRightVelCached() {
    if (Double.isNaN(this.cachedRightVel)) {
      this.cachedRightVel = this.getRightVel();
    }
    return this.cachedRightVel;
  }

  @Override
  public double getLeftPosCached() {
    if (Double.isNaN(this.cachedLeftPos)) {
      this.cachedLeftPos = this.getLeftPos();
    }
    return this.cachedLeftPos;
  }

  @Override
  public double getRightPosCached() {
    if (Double.isNaN(this.cachedRightPos)) {
      this.cachedRightPos = this.getRightPos();
    }
    return this.cachedRightPos;
  }

  /**
   * @return The feedforward calculator for left motors
   */
  public SimpleMotorFeedforward getFeedforward() {
    return settings.feedforward;
  }

  /** The left PID velocity controller */
  public PIDController leftVelPID() {
    return settings.leftVelPID;
  }

  /** The right PID velocity controller */
  public PIDController rightVelPID() {
    return settings.rightVelPID;
  }

  /** Updates all cached values with current ones. */
  @Override
  public void periodic() {
    this.cachedLeftVel = this.getLeftVel();
    this.cachedLeftPos = this.getLeftPos();
    this.cachedRightVel = this.getRightVel();
    this.cachedRightPos = this.getRightPos();
  }
}
