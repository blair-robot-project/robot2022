package frc.team449.javaMaps.builders;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.team449.generalInterfaces.DriveSettings;

public final class DriveSettingsBuilder {
  private Double fwdPeakOutputVoltage;
  private Double revPeakOutputVoltage;
  private Double fwdNominalOutputVoltage;
  private Double revNominalOutputVoltage;
  private SimpleMotorFeedforward leftFeedforward;
  private SimpleMotorFeedforward rightFeedforward;
  private PIDController leftPosPID;
  private PIDController rightPosPID;
  private PIDController leftVelPID;
  private PIDController rightVelPID;
  private Double rampRate;
  private Double maxSpeed;

  public DriveSettingsBuilder copy() {
    return new DriveSettingsBuilder()
        .fwdPeakOutputVoltage(fwdPeakOutputVoltage)
        .revPeakOutputVoltage(revPeakOutputVoltage)
        .fwdNominalOutputVoltage(fwdNominalOutputVoltage)
        .revNominalOutputVoltage(revNominalOutputVoltage)
        .leftFeedforward(leftFeedforward)
        .rightFeedforward(rightFeedforward)
        .leftPosPID(leftPosPID)
        .rightPosPID(rightPosPID)
        .leftVelPID(leftVelPID)
        .rightVelPID(rightVelPID)
        .rampRate(rampRate)
        .maxSpeed(maxSpeed);
  }

  public DriveSettings build() {
    return new DriveSettings(
        leftFeedforward,
        rightFeedforward,
        leftPosPID,
        rightPosPID,
        leftVelPID,
        rightVelPID,
        rampRate,
        maxSpeed);
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

  public DriveSettingsBuilder leftFeedforward(SimpleMotorFeedforward leftFeedforward) {
    this.leftFeedforward = leftFeedforward;
    return this;
  }

  public DriveSettingsBuilder rightFeedforward(SimpleMotorFeedforward rightFeedforward) {
    this.rightFeedforward = rightFeedforward;
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
}
