package frc.team449.javaMaps.builders;

import frc.team449.generalInterfaces.SmartMotor;
import frc.team449.generalInterfaces.shiftable.Shiftable;
import frc.team449.jacksonWrappers.PDP;
import frc.team449.jacksonWrappers.SlaveSparkMax;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SmartMotorConfig {
  public final SmartMotor.Type type;
  public final int port;
  public final boolean enableBrakeMode;
  public final @Nullable String name;
  public final boolean reverseOutput;
  public final @NotNull PDP pdp;
  public final @Nullable Boolean fwdLimitSwitchNormallyOpen;
  public final @Nullable Boolean revLimitSwitchNormallyOpen;
  public final @Nullable Integer remoteLimitSwitchID;
  public final @Nullable Double fwdSoftLimit;
  public final @Nullable Double revSoftLimit;
  public final double postEncoderGearing;
  public final double unitPerRotation;
  public final @Nullable Integer currentLimit;
  public final boolean enableVoltageComp;
  public final @NotNull Map<Integer, Shiftable.PerGearSettings> perGearSettings;
  public final Shiftable.Gear startingGear;
  public final Integer startingGearNum;
  public final @NotNull List<SlaveSparkMax> slaveSparks;

  public final Shiftable.PerGearSettings initialGearSettings;

  public SmartMotorConfig(
      final @NotNull SmartMotor.Type type,
      final int port,
      final boolean enableBrakeMode,
      final @Nullable String name,
      final boolean reverseOutput,
      final @NotNull PDP pdp,
      final @Nullable Boolean fwdLimitSwitchNormallyOpen,
      final @Nullable Boolean revLimitSwitchNormallyOpen,
      final @Nullable Integer remoteLimitSwitchID,
      final @Nullable Double fwdSoftLimit,
      final @Nullable Double revSoftLimit,
      final @Nullable Double postEncoderGearing,
      final @Nullable Double unitPerRotation,
      final @Nullable Integer currentLimit,
      final boolean enableVoltageComp,
      final @NotNull List<Shiftable.PerGearSettings> perGearSettings,
      final @Nullable Shiftable.Gear startingGear,
      final @Nullable Integer startingGearNum,
      final @NotNull List<SlaveSparkMax> slaveSparks) {

    this.type = type;
    this.port = port;
    this.enableBrakeMode = enableBrakeMode;
    this.name = name;
    this.reverseOutput = reverseOutput;
    this.pdp = pdp;
    this.fwdLimitSwitchNormallyOpen = fwdLimitSwitchNormallyOpen;
    this.revLimitSwitchNormallyOpen = revLimitSwitchNormallyOpen;
    this.remoteLimitSwitchID = remoteLimitSwitchID;
    this.fwdSoftLimit = fwdSoftLimit;
    this.revSoftLimit = revSoftLimit;
    this.currentLimit = currentLimit;
    this.enableVoltageComp = enableVoltageComp;
    this.startingGear = startingGear;
    this.startingGearNum = startingGearNum;
    this.slaveSparks = slaveSparks;

    this.unitPerRotation = unitPerRotation == null ? 1 : unitPerRotation;
    this.postEncoderGearing = postEncoderGearing == null ? 1 : postEncoderGearing;
    this.perGearSettings = new HashMap<>();

    // If given no gear settings, use the default values.
    if (perGearSettings.isEmpty()) {
      this.perGearSettings.put(0, new Shiftable.PerGearSettings());
    }
    // Otherwise, map the settings to the gear they are.
    else {
      for (final Shiftable.PerGearSettings settings : perGearSettings) {
        this.perGearSettings.put(settings.gear, settings);
      }
    }

    int currentGear;
    // If the starting gear isn't given, assume we start in low gear.
    if (startingGear == null) {
      if (startingGearNum == null) {
        currentGear = Integer.MAX_VALUE;
        for (final Integer gear : this.perGearSettings.keySet()) {
          if (gear < currentGear) {
            currentGear = gear;
          }
        }
      } else {
        currentGear = startingGearNum;
      }
    } else {
      currentGear = startingGear.getNumVal();
    }
    this.initialGearSettings = this.perGearSettings.get(currentGear);
  }
}
