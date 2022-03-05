package frc.team449.drive.unidirectional;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.drive.DriveSettings;
import frc.team449.other.Updater;
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
    //todo add feedforward
    this.leftMaster.set(left);
    this.rightMaster.set(right);
  }

  /**
   * Set the position setpoint for both the left and right sides. Note: This actually moves there
   *
   * @param leftPos Position setpoint for left side
   * @param rightPos Position setpoint for right side
   */
  public void setPositionSetpoint(double leftPos, double rightPos) {
    var leftOutput = settings.leftPosPID.calculate(this.getLeftPos(), leftPos);
    var rightOutput = settings.rightPosPID.calculate(this.getRightPos(), rightPos);

    while (!settings.leftPosPID.atSetpoint() || !settings.rightPosPID.atSetpoint()) {
      this.setOutput(leftOutput, rightOutput);
      leftOutput = settings.leftPosPID.calculate(this.getLeftPos());
      rightOutput = settings.rightPosPID.calculate(this.getRightPos());
    }
  }

  /**
   * Hold the current position.
   *
   * @param pos the position to stop at
   */
  public void holdPosition(double pos) {
    this.holdPosition(pos, pos);
  }

  /**
   * Hold the current position.
   *
   * @param leftPos the position to stop the left side at
   * @param rightPos the position to stop the right side at
   */
  public void holdPosition(double leftPos, double rightPos) {
    this.setPositionSetpoint(leftPos, rightPos);
  }

  @NotNull
  @Override
  public Double getLeftVel() {
    return this.leftMaster.encoder.getVelocityUnits();
  }

  @NotNull
  @Override
  public Double getRightVel() {
    return this.rightMaster.encoder.getVelocityUnits();
  }

  @NotNull
  @Override
  public Double getLeftPos() {
    return this.leftMaster.encoder.getPositionUnits();
  }

  @NotNull
  @Override
  public Double getRightPos() {
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

  @NotNull
  @Override
  public Double getLeftPosCached() {
    return this.cachedLeftPos;
  }

  @NotNull
  @Override
  public Double getRightPosCached() {
    return this.cachedRightPos;
  }

  /** @return The feedforward calculator for left motors */
  public SimpleMotorFeedforward getFeedforward() {
    return settings.feedforward;
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
