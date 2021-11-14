package frc.team449.javaMaps.builders;

import frc.team449.generalInterfaces.SmartMotor;
import frc.team449.generalInterfaces.shiftable.Shiftable;
import frc.team449.jacksonWrappers.PDP;
import frc.team449.jacksonWrappers.SlaveSparkMax;
import java.util.ArrayList;
import java.util.List;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * The constructor for {@link SmartMotor} was hell so this will help resolve that.
 *
 * <p>You can set config options in this, then build a {@link SmartMotorConfig} from it to pass to
 * various constructors.
 *
 * @author Katie Del Toro
 * @see SmartMotor
 */
public class SmartMotorConfigBuilder {
  private SmartMotor.Type type;
  private int port;
  private boolean enableBrakeMode;
  private @Nullable String name;
  private boolean reverseOutput;
  private @Nullable PDP pdp;
  private @Nullable Boolean fwdLimitSwitchNormallyOpen;
  private @Nullable Boolean revLimitSwitchNormallyOpen;
  private @Nullable Integer remoteLimitSwitchID;
  private @Nullable Double fwdSoftLimit;
  private @Nullable Double revSoftLimit;
  private @Nullable Double postEncoderGearing;
  private @Nullable Double unitPerRotation;
  private @Nullable Integer currentLimit;
  private boolean enableVoltageComp;
  private @Nullable List<Shiftable.PerGearSettings> perGearSettings = new ArrayList<>();
  private @Nullable Shiftable.Gear startingGear;
  private @Nullable Integer startingGearNum;
  private @NotNull List<SlaveSparkMax> slaveSparks = new ArrayList<>();

  public SmartMotor.Type getType() {
    return type;
  }

  public SmartMotorConfigBuilder setType(SmartMotor.Type type) {
    this.type = type;
    return this;
  }

  public int getPort() {
    return port;
  }

  public SmartMotorConfigBuilder setPort(int port) {
    this.port = port;
    return this;
  }

  public boolean isEnableBrakeMode() {
    return enableBrakeMode;
  }

  public SmartMotorConfigBuilder setEnableBrakeMode(boolean enableBrakeMode) {
    this.enableBrakeMode = enableBrakeMode;
    return this;
  }

  public @Nullable String getName() {
    return name;
  }

  public SmartMotorConfigBuilder setName(String name) {
    this.name = name;
    return this;
  }

  public boolean isReverseOutput() {
    return reverseOutput;
  }

  public SmartMotorConfigBuilder setReverseOutput(boolean reverseOutput) {
    this.reverseOutput = reverseOutput;
    return this;
  }

  public @Nullable PDP getPdp() {
    return pdp;
  }

  public SmartMotorConfigBuilder setPdp(PDP pdp) {
    this.pdp = pdp;
    return this;
  }

  public @Nullable Boolean getFwdLimitSwitchNormallyOpen() {
    return fwdLimitSwitchNormallyOpen;
  }

  public SmartMotorConfigBuilder setFwdLimitSwitchNormallyOpen(Boolean fwdLimitSwitchNormallyOpen) {
    this.fwdLimitSwitchNormallyOpen = fwdLimitSwitchNormallyOpen;
    return this;
  }

  public @Nullable Boolean getRevLimitSwitchNormallyOpen() {
    return revLimitSwitchNormallyOpen;
  }

  public SmartMotorConfigBuilder setRevLimitSwitchNormallyOpen(Boolean revLimitSwitchNormallyOpen) {
    this.revLimitSwitchNormallyOpen = revLimitSwitchNormallyOpen;
    return this;
  }

  public @Nullable Integer getRemoteLimitSwitchID() {
    return remoteLimitSwitchID;
  }

  public SmartMotorConfigBuilder setRemoteLimitSwitchID(Integer remoteLimitSwitchID) {
    this.remoteLimitSwitchID = remoteLimitSwitchID;
    return this;
  }

  public @Nullable Double getFwdSoftLimit() {
    return fwdSoftLimit;
  }

  public SmartMotorConfigBuilder setFwdSoftLimit(Double fwdSoftLimit) {
    this.fwdSoftLimit = fwdSoftLimit;
    return this;
  }

  public @Nullable Double getRevSoftLimit() {
    return revSoftLimit;
  }

  public SmartMotorConfigBuilder setRevSoftLimit(Double revSoftLimit) {
    this.revSoftLimit = revSoftLimit;
    return this;
  }

  public @Nullable Double getPostEncoderGearing() {
    return postEncoderGearing;
  }

  public SmartMotorConfigBuilder setPostEncoderGearing(Double postEncoderGearing) {
    this.postEncoderGearing = postEncoderGearing;
    return this;
  }

  public @Nullable Double getUnitPerRotation() {
    return unitPerRotation;
  }

  public SmartMotorConfigBuilder setUnitPerRotation(Double unitPerRotation) {
    this.unitPerRotation = unitPerRotation;
    return this;
  }

  public @Nullable Integer getCurrentLimit() {
    return currentLimit;
  }

  public SmartMotorConfigBuilder setCurrentLimit(Integer currentLimit) {
    this.currentLimit = currentLimit;
    return this;
  }

  public boolean isEnableVoltageComp() {
    return enableVoltageComp;
  }

  public SmartMotorConfigBuilder setEnableVoltageComp(boolean enableVoltageComp) {
    this.enableVoltageComp = enableVoltageComp;
    return this;
  }

  public @Nullable List<Shiftable.PerGearSettings> getPerGearSettings() {
    return perGearSettings;
  }

  public SmartMotorConfigBuilder setPerGearSettings(
      List<Shiftable.PerGearSettings> perGearSettings) {
    this.perGearSettings = perGearSettings;
    return this;
  }

  public Shiftable.@Nullable Gear getStartingGear() {
    return startingGear;
  }

  public SmartMotorConfigBuilder setStartingGear(Shiftable.Gear startingGear) {
    this.startingGear = startingGear;
    return this;
  }

  public @Nullable Integer getStartingGearNum() {
    return startingGearNum;
  }

  public SmartMotorConfigBuilder setStartingGearNum(Integer startingGearNum) {
    this.startingGearNum = startingGearNum;
    return this;
  }

  public @NotNull List<SlaveSparkMax> getSlaveSparks() {
    return slaveSparks;
  }

  public SmartMotorConfigBuilder setSlaveSparks(@NotNull List<SlaveSparkMax> slaveSparks) {
    this.slaveSparks = slaveSparks;
    return this;
  }

  public SmartMotorConfigBuilder copy() {
    return new SmartMotorConfigBuilder()
        .setType(type)
        .setPort(port)
        .setEnableBrakeMode(enableBrakeMode)
        .setName(name)
        .setReverseOutput(reverseOutput)
        .setPdp(pdp)
        .setFwdLimitSwitchNormallyOpen(fwdLimitSwitchNormallyOpen)
        .setRevLimitSwitchNormallyOpen(revLimitSwitchNormallyOpen)
        .setRemoteLimitSwitchID(remoteLimitSwitchID)
        .setFwdSoftLimit(fwdSoftLimit)
        .setRevSoftLimit(revSoftLimit)
        .setPostEncoderGearing(postEncoderGearing)
        .setUnitPerRotation(unitPerRotation)
        .setCurrentLimit(currentLimit)
        .setEnableVoltageComp(enableVoltageComp)
        .setPerGearSettings(perGearSettings)
        .setStartingGear(startingGear)
        .setStartingGearNum(startingGearNum)
        .setSlaveSparks(slaveSparks);
  }

  public SmartMotorConfig build() {
    assert pdp != null : "PDP was null when constructing motor " + name;
    return new SmartMotorConfig(
        type,
        port,
        enableBrakeMode,
        name,
        reverseOutput,
        pdp,
        fwdLimitSwitchNormallyOpen,
        revLimitSwitchNormallyOpen,
        remoteLimitSwitchID,
        fwdSoftLimit,
        revSoftLimit,
        postEncoderGearing,
        unitPerRotation,
        currentLimit,
        enableVoltageComp,
        perGearSettings == null ? List.of() : perGearSettings,
        startingGear,
        startingGearNum,
        slaveSparks);
  }
}
