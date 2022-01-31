package frc.team449.javaMaps.builders;

import edu.wpi.first.hal.util.HalHandleException;
import edu.wpi.first.wpilibj.Encoder;
import frc.team449.jacksonWrappers.SlaveSparkMax;
import frc.team449.jacksonWrappers.WrappedMotor;
import frc.team449.jacksonWrappers.simulated.DummyMotorController;
import frc.team449.jacksonWrappers.simulated.DummyWrappedEncoder;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;

/**
 * The constructor for SmartMotors was hell so this will help resolve that.
 *
 * <p>You can set config options in this then pass it to various constructors.
 *
 * <p>todo find a better way to make the subclass the return type than F-bounds
 *
 * @param <Self> The type of the "current" subclass of {@link MotorConfig}
 * @see frc.team449.jacksonWrappers.WrappedMotor
 */
@SuppressWarnings({"unchecked", "UnusedReturnValue"})
public abstract class MotorConfig<Self extends MotorConfig<Self>> {
  private int port;
  private boolean enableBrakeMode;
  private @Nullable String name;
  private boolean reverseOutput;
  private @Nullable Boolean fwdLimitSwitchNormallyOpen;
  private @Nullable Boolean revLimitSwitchNormallyOpen;
  private @Nullable Integer remoteLimitSwitchID;
  private @Nullable Double fwdSoftLimit;
  private @Nullable Double revSoftLimit;
  private double postEncoderGearing = 1.0;
  private double unitPerRotation = 1.0;
  private int encoderCPR = 1;
  private @Nullable Double rampRate;
  private @Nullable Integer currentLimit;
  private boolean enableVoltageComp;
  private @NotNull List<SlaveSparkMax> slaveSparks = new ArrayList<>();
  private @Nullable Encoder externalEncoder;

  public int getPort() {
    return port;
  }

  public Self setPort(int port) {
    this.port = port;
    return (Self) this;
  }

  public boolean isEnableBrakeMode() {
    return enableBrakeMode;
  }

  public Self setEnableBrakeMode(boolean enableBrakeMode) {
    this.enableBrakeMode = enableBrakeMode;
    return (Self) this;
  }

  public @Nullable String getName() {
    return name;
  }

  public Self setName(@NotNull String name) {
    this.name = name;
    return (Self) this;
  }

  public boolean isReverseOutput() {
    return reverseOutput;
  }

  public Self setReverseOutput(boolean reverseOutput) {
    this.reverseOutput = reverseOutput;
    return (Self) this;
  }

  public @Nullable Boolean getFwdLimitSwitchNormallyOpen() {
    return fwdLimitSwitchNormallyOpen;
  }

  public Self setFwdLimitSwitchNormallyOpen(Boolean fwdLimitSwitchNormallyOpen) {
    this.fwdLimitSwitchNormallyOpen = fwdLimitSwitchNormallyOpen;
    return (Self) this;
  }

  public @Nullable Boolean getRevLimitSwitchNormallyOpen() {
    return revLimitSwitchNormallyOpen;
  }

  public Self setRevLimitSwitchNormallyOpen(Boolean revLimitSwitchNormallyOpen) {
    this.revLimitSwitchNormallyOpen = revLimitSwitchNormallyOpen;
    return (Self) this;
  }

  public @Nullable Integer getRemoteLimitSwitchID() {
    return remoteLimitSwitchID;
  }

  public Self setRemoteLimitSwitchID(Integer remoteLimitSwitchID) {
    this.remoteLimitSwitchID = remoteLimitSwitchID;
    return (Self) this;
  }

  public @Nullable Double getFwdSoftLimit() {
    return fwdSoftLimit;
  }

  public Self setFwdSoftLimit(Double fwdSoftLimit) {
    this.fwdSoftLimit = fwdSoftLimit;
    return (Self) this;
  }

  public @Nullable Double getRevSoftLimit() {
    return revSoftLimit;
  }

  public Self setRevSoftLimit(Double revSoftLimit) {
    this.revSoftLimit = revSoftLimit;
    return (Self) this;
  }

  /** The counts per rotation of the encoder being used, if applicable */
  public int getEncoderCPR() {
    return encoderCPR;
  }

  public Self setEncoderCPR(int encoderCPR) {
    this.encoderCPR = encoderCPR;
    return (Self) this;
  }

  public double getPostEncoderGearing() {
    return postEncoderGearing;
  }

  public Self setPostEncoderGearing(double postEncoderGearing) {
    this.postEncoderGearing = postEncoderGearing;
    return (Self) this;
  }

  public double getUnitPerRotation() {
    return unitPerRotation;
  }

  public Self setUnitPerRotation(double unitPerRotation) {
    this.unitPerRotation = unitPerRotation;
    return (Self) this;
  }

  /** The ramp rate, in volts/sec. null means no ramp rate. */
  public @Nullable Double getRampRate() {
    return rampRate;
  }

  /** Set the ramp rate in volts/sec. Use {@code null} for no ramp rate */
  public Self setRampRate(Double rampRate) {
    this.rampRate = rampRate;
    return (Self) this;
  }

  public @Nullable Integer getCurrentLimit() {
    return currentLimit;
  }

  public Self setCurrentLimit(Integer currentLimit) {
    this.currentLimit = currentLimit;
    return (Self) this;
  }

  public boolean isEnableVoltageComp() {
    return enableVoltageComp;
  }

  public Self setEnableVoltageComp(boolean enableVoltageComp) {
    this.enableVoltageComp = enableVoltageComp;
    return (Self) this;
  }

  public @NotNull List<SlaveSparkMax> getSlaveSparks() {
    return slaveSparks;
  }

  public Self addSlaveSpark(@NotNull SlaveSparkMax slaveSpark) {
    this.slaveSparks.add(slaveSpark);
    return (Self) this;
  }

  /**
   * Return the external encoder, if it exists. Returns {@code null} if the integrated encoder is to
   * be used.
   */
  @Nullable
  public Encoder getExternalEncoder() {
    return this.externalEncoder;
  }

  /** Use an external encoder instead of the integrated encoder */
  public Self setExternalEncoder(@NotNull Encoder externalEncoder) {
    this.externalEncoder = externalEncoder;
    return (Self) this;
  }

  /** Copy properties from this config to another config */
  protected void copyTo(MotorConfig<?> other) {
    other
        .setPort(port)
        .setEnableBrakeMode(enableBrakeMode)
        .setReverseOutput(reverseOutput)
        .setFwdLimitSwitchNormallyOpen(fwdLimitSwitchNormallyOpen)
        .setRevLimitSwitchNormallyOpen(revLimitSwitchNormallyOpen)
        .setRemoteLimitSwitchID(remoteLimitSwitchID)
        .setFwdSoftLimit(fwdSoftLimit)
        .setRevSoftLimit(revSoftLimit)
        .setEncoderCPR(encoderCPR)
        .setPostEncoderGearing(postEncoderGearing)
        .setUnitPerRotation(unitPerRotation)
        .setRampRate(rampRate)
        .setCurrentLimit(currentLimit)
        .setEnableVoltageComp(enableVoltageComp);

    for (var slave : this.slaveSparks) {
      other.addSlaveSpark(slave);
    }

    if (this.name != null) other.setName(name);
    if (this.externalEncoder != null) other.setExternalEncoder(externalEncoder);
  }

  /** Create a simulated motor */
  @NotNull
  public WrappedMotor createSim() {
    return new WrappedMotor(
        "sim_" + port,
        new DummyMotorController(),
        new DummyWrappedEncoder(encoderCPR, unitPerRotation, postEncoderGearing));
  }

  /** Try creating a real motor, and if that fails, create a simulated one */
  @NotNull
  public WrappedMotor createRealOrSim() {
    try {
      return this.createReal();
    } catch (HalHandleException e) {
      return this.createSim();
    }
  }

  /**
   * Create a physical motor
   *
   * @throws HalHandleException if the motor couldn't be constructed
   */
  public abstract WrappedMotor createReal();
}
