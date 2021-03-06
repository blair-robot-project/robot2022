package frc.team449.motor.builder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.team449.motor.WrappedMotor;
import frc.team449.motor.sim.DummyMotorController;
import frc.team449.motor.sim.SimulatedEncoder;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * The constructor for SmartMotors was hell so this will help resolve that.
 *
 * <p>You can set config options in this then pass it to various constructors.
 *
 * <p>todo find a better way to make the subclass the return type than F-bounds
 *
 * @param <Self> The type of the "current" subclass of {@link MotorConfig}
 * @see WrappedMotor
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
  private @Nullable Encoder externalEncoder;
  private boolean calculateVel = false;

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

  public boolean getCalculateVel() {
    return this.calculateVel;
  }

  /**
   * Whether or not to calculate velocity ourselves instead of using the encoder's builtin method
   */
  public Self setCalculateVel(boolean calculateVel) {
    this.calculateVel = calculateVel;
    return (Self) this;
  }

  /** Copy properties from this config to another config */
  protected final void copyTo(@NotNull MotorConfig<?> other) {
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
        .setEnableVoltageComp(enableVoltageComp)
        .setCalculateVel(calculateVel);

    if (this.name != null) other.setName(name);
    if (this.externalEncoder != null) other.setExternalEncoder(externalEncoder);
  }

  /**
   * Create a simulated motor
   *
   * @param encSim An object used for simulating the encoder (optional)
   */
  @NotNull
  public WrappedMotor createSim(@Nullable EncoderSim encSim) {
    if (encSim == null) {
      var extEnc = externalEncoder == null ? new Encoder(0, 0) : externalEncoder;
      encSim = new EncoderSim(extEnc);
    }
    var name = this.name == null ? "motor_" + port : this.name;
    return new WrappedMotor(
        "sim_" + name,
        new DummyMotorController(),
        new SimulatedEncoder(
            "sim_enc_" + name, encSim, encoderCPR, unitPerRotation, postEncoderGearing));
  }

  /**
   * Create a real motor if not in a simulation and a simulated motor otherwise.
   *
   * @param encSim An object used for simulating the encoder (optional)
   */
  @NotNull
  public WrappedMotor createRealOrSim(@Nullable EncoderSim encSim) {
    if (RobotBase.isReal()) {
      return this.createReal();
    } else {
      return this.createSim(encSim);
    }
  }

  /** Create a physical motor */
  @Contract("-> new")
  @NotNull
  public abstract WrappedMotor createReal();
}
