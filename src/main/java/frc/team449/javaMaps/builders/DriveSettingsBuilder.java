package frc.team449.javaMaps.builders;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.team449.generalInterfaces.DriveSettings;

public final class DriveSettingsBuilder {
  private Double fwdPeakOutputVoltage;
  private Double revPeakOutputVoltage;
  private Double fwdNominalOutputVoltage;
  private Double revNominalOutputVoltage;
  private SimpleMotorFeedforward feedforward;
  private PIDController leftPosPID;
  private PIDController rightPosPID;
  private PIDController leftVelPID;
  private PIDController rightVelPID;
  private Double rampRate;
  private Double maxSpeed;
  private Double trackWidth;

  public DriveSettingsBuilder copy() {
    var copy = new DriveSettingsBuilder()
        .fwdPeakOutputVoltage(fwdPeakOutputVoltage)
        .revPeakOutputVoltage(revPeakOutputVoltage)
        .fwdNominalOutputVoltage(fwdNominalOutputVoltage)
        .revNominalOutputVoltage(revNominalOutputVoltage)
        .feedforward(feedforward)
        .leftPosPID(leftPosPID)
        .rightPosPID(rightPosPID)
        .leftVelPID(leftVelPID)
        .rightVelPID(rightVelPID)
        .rampRate(rampRate)
        .maxSpeed(maxSpeed);
    if (this.trackWidth != null) {
      copy.trackWidth(this.trackWidth);
    }
    return copy;
  }

  public DriveSettings build() {
    return new DriveSettings(
        feedforward,
        leftPosPID,
        rightPosPID,
        leftVelPID,
        rightVelPID,
        rampRate,
        maxSpeed,
        trackWidth);
  }

  public DriveSettingsBuilder fwdPeakOutputVoltage(Double fwdPeakOutputVoltage) {
    this.fwdPeakOutputVoltage = fwdPeakOutputVoltage;
    return this;
  }

  public DriveSettingsBuilder revPeakOutputVoltage(Double revPeakOutputVoltage) {
    this.revPeakOutputVoltage = revPeakOutputVoltage;
    return this;
  }

  public DriveSettingsBuilder fwdNominalOutputVoltage(Double fwdNominalOutputVoltage) {
    this.fwdNominalOutputVoltage = fwdNominalOutputVoltage;
    return this;
  }

  public DriveSettingsBuilder revNominalOutputVoltage(Double revNominalOutputVoltage) {
    this.revNominalOutputVoltage = revNominalOutputVoltage;
    return this;
  }

  public DriveSettingsBuilder feedforward(SimpleMotorFeedforward feedforward) {
    this.feedforward = feedforward;
    return this;
  }

  public DriveSettingsBuilder leftPosPID(PIDController leftPosPID) {
    this.leftPosPID = leftPosPID;
    return this;
  }

  public DriveSettingsBuilder rightPosPID(PIDController rightPosPID) {
    this.rightPosPID = rightPosPID;
    return this;
  }

  public DriveSettingsBuilder leftVelPID(PIDController leftVelPID) {
    this.leftVelPID = leftVelPID;
    return this;
  }

  public DriveSettingsBuilder rightVelPID(PIDController rightVelPID) {
    this.rightVelPID = rightVelPID;
    return this;
  }

  public DriveSettingsBuilder rampRate(Double rampRate) {
    this.rampRate = rampRate;
    return this;
  }

  public DriveSettingsBuilder maxSpeed(Double maxSpeed) {
    this.maxSpeed = maxSpeed;
    return this;
  }

  public DriveSettingsBuilder trackWidth(double trackWidth) {
    this.trackWidth = trackWidth;
    return this;
  }
}
