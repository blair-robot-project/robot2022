package frc.team449.javaMaps.builders;

import frc.team449.generalInterfaces.SmartMotor;
import frc.team449.generalInterfaces.shiftable.Shiftable;
import frc.team449.jacksonWrappers.PDP;
import frc.team449.jacksonWrappers.SlaveSparkMax;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * The constructor for {@link SmartMotor} was hell so this will help resolve that.
 *
 * <p>You can set config options in this then pass it to various constructors.
 *
 * @author Katie Del Toro
 * @see SmartMotor
 */
public class SmartMotorConfig {
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
  private double postEncoderGearing = 1.0;
  private double unitPerRotation = 1.0;
  private @Nullable Integer currentLimit;
  private boolean enableVoltageComp;
  private @NotNull List<Shiftable.PerGearSettings> perGearSettings = new ArrayList<>();
  private @Nullable Shiftable.Gear startingGear;
  private @Nullable Integer startingGearNum;
  private @NotNull List<SlaveSparkMax> slaveSparks = new ArrayList<>();

  public SmartMotor.Type getType() {
    return type;
  }

  public SmartMotorConfig setType(SmartMotor.Type type) {
    this.type = type;
    return this;
  }

  public int getPort() {
    return port;
  }

  public SmartMotorConfig setPort(int port) {
    this.port = port;
    return this;
  }

  public boolean isEnableBrakeMode() {
    return enableBrakeMode;
  }

  public SmartMotorConfig setEnableBrakeMode(boolean enableBrakeMode) {
    this.enableBrakeMode = enableBrakeMode;
    return this;
  }

  public @Nullable String getName() {
    return name;
  }

  public SmartMotorConfig setName(@NotNull String name) {
    this.name = name;
    return this;
  }

  public boolean isReverseOutput() {
    return reverseOutput;
  }

  public SmartMotorConfig setReverseOutput(boolean reverseOutput) {
    this.reverseOutput = reverseOutput;
    return this;
  }

  public @Nullable PDP getPdp() {
    return pdp;
  }

  public SmartMotorConfig setPdp(@NotNull PDP pdp) {
    this.pdp = pdp;
    return this;
  }

  public @Nullable Boolean getFwdLimitSwitchNormallyOpen() {
    return fwdLimitSwitchNormallyOpen;
  }

  public SmartMotorConfig setFwdLimitSwitchNormallyOpen(Boolean fwdLimitSwitchNormallyOpen) {
    this.fwdLimitSwitchNormallyOpen = fwdLimitSwitchNormallyOpen;
    return this;
  }

  public @Nullable Boolean getRevLimitSwitchNormallyOpen() {
    return revLimitSwitchNormallyOpen;
  }

  public SmartMotorConfig setRevLimitSwitchNormallyOpen(Boolean revLimitSwitchNormallyOpen) {
    this.revLimitSwitchNormallyOpen = revLimitSwitchNormallyOpen;
    return this;
  }

  public @Nullable Integer getRemoteLimitSwitchID() {
    return remoteLimitSwitchID;
  }

  public SmartMotorConfig setRemoteLimitSwitchID(Integer remoteLimitSwitchID) {
    this.remoteLimitSwitchID = remoteLimitSwitchID;
    return this;
  }

  public @Nullable Double getFwdSoftLimit() {
    return fwdSoftLimit;
  }

  public SmartMotorConfig setFwdSoftLimit(Double fwdSoftLimit) {
    this.fwdSoftLimit = fwdSoftLimit;
    return this;
  }

  public @Nullable Double getRevSoftLimit() {
    return revSoftLimit;
  }

  public SmartMotorConfig setRevSoftLimit(Double revSoftLimit) {
    this.revSoftLimit = revSoftLimit;
    return this;
  }

  public double getPostEncoderGearing() {
    return postEncoderGearing;
  }

  public SmartMotorConfig setPostEncoderGearing(double postEncoderGearing) {
    this.postEncoderGearing = postEncoderGearing;
    return this;
  }

  public double getUnitPerRotation() {
    return unitPerRotation;
  }

  public SmartMotorConfig setUnitPerRotation(double unitPerRotation) {
    this.unitPerRotation = unitPerRotation;
    return this;
  }

  public @Nullable Integer getCurrentLimit() {
    return currentLimit;
  }

  public SmartMotorConfig setCurrentLimit(Integer currentLimit) {
    this.currentLimit = currentLimit;
    return this;
  }

  public boolean isEnableVoltageComp() {
    return enableVoltageComp;
  }

  public SmartMotorConfig setEnableVoltageComp(boolean enableVoltageComp) {
    this.enableVoltageComp = enableVoltageComp;
    return this;
  }

  public SmartMotorConfig setPerGearSettings(
      @NotNull List<Shiftable.PerGearSettings> perGearSettings) {
    this.perGearSettings = perGearSettings;
    return this;
  }

  public SmartMotorConfig setStartingGear(Shiftable.Gear startingGear) {
    this.startingGear = startingGear;
    return this;
  }

  public SmartMotorConfig setStartingGearNum(Integer startingGearNum) {
    this.startingGearNum = startingGearNum;
    return this;
  }

  public @NotNull List<SlaveSparkMax> getSlaveSparks() {
    return slaveSparks;
  }

  public SmartMotorConfig setSlaveSparks(@NotNull List<SlaveSparkMax> slaveSparks) {
    this.slaveSparks = slaveSparks;
    return this;
  }

  public SmartMotorConfig copy() {
    return new SmartMotorConfig()
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

  public Map<Integer, Shiftable.PerGearSettings> getPerGearSettingsMap() {
    // If given no gear settings, use the default values.
    if (this.perGearSettings.isEmpty()) {
      return Map.of(0, Shiftable.PerGearSettings.DEFAULT);
    }
    // Otherwise, map the settings to the gear they are.
    return perGearSettings.stream()
        .collect(Collectors.toMap(settings -> settings.gear, settings -> settings));
  }

  public Shiftable.PerGearSettings getInitialGearSettings() {
    // If the starting gear isn't given, assume we start in low gear.

    if (startingGear != null || startingGearNum != null) {
      var currentGear = startingGear != null ? startingGear.getNumVal() : startingGearNum;
      for (var settings : this.perGearSettings) {
        if (settings.gear == currentGear) {
          return settings;
        }
      }
      throw new RuntimeException("No PerGearSettings corresponding to gear number " + currentGear);
    } else {
      return this.perGearSettings.stream()
          .min(Comparator.comparingInt(settings -> settings.gear))
          .orElse(Shiftable.PerGearSettings.DEFAULT);
    }
  }

  /** Ensure that all required fields of this {@link SmartMotorConfig} have been initialized */
  public SmartMotorConfig ensureBuilt() {
    assert pdp != null : "PDP was null when constructing motor " + name;
    return this;
  }
}
