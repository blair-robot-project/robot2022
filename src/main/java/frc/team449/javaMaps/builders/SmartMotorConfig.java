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

/**
 * This class holds settings that are common to both Talons and Sparks. To create an instance of
 * this class, use {@link SmartMotorConfigBuilder} instead of the constructor itself.
 */
public class SmartMotorConfig {
  /** The type of controller to create. */
  public final SmartMotor.Type type;
  /** The controller's CAN port */
  public final int port;
  /** Whether to brake or coast when stopped */
  public final boolean enableBrakeMode;
  /** This controller's name */
  public final @Nullable String name;
  /** Whether or not to reverse the motor */
  public final boolean reverseOutput;
  /** The PDP this controller is connected to. */
  public final @NotNull PDP pdp;
  /**
   * Whether the forward limit switch is normally open or closed. If this is null, the forward limit
   * switch is disabled.
   */
  public final @Nullable Boolean fwdLimitSwitchNormallyOpen;
  /**
   * Whether the reverse limit switch is normally open or closed. If this is null, the reverse limit
   * switch is disabled.
   */
  public final @Nullable Boolean revLimitSwitchNormallyOpen;
  /**
   * The CAN port that the limit switch to use for this controller is plugged into, or null to not
   * use a limit switch or use the limit switch plugged directly into this controller (for some
   * controllers).
   */
  public final @Nullable Integer remoteLimitSwitchID;
  /**
   * The forward software limit, in meters. If this is null, the forward software limit is disabled.
   * Ignored if there's no encoder.
   */
  public final @Nullable Double fwdSoftLimit;
  /**
   * The reverse software limit, in meters. If this is null, the reverse software limit is disabled.
   * Ignored if there's no encoder.
   */
  public final @Nullable Double revSoftLimit;
  /**
   * The coefficient the output changes by after being measured by the encoder, e.g. this would be
   * 1/70 if there was a 70:1 gearing between the encoder and the final output. Defaults to 1.
   */
  public final double postEncoderGearing;
  /**
   * The number of meters travelled per rotation of the motor this is attached to. Defaults to 1.
   */
  public final double unitPerRotation;
  /** The max amps this device can draw. If this is null, no current limit is used. */
  public final @Nullable Integer currentLimit;
  /** Whether or not to use voltage compensation. Defaults to false. */
  public final boolean enableVoltageComp;
  /**
   * The settings for each gear this motor has. Can be null to use default values and gear # of
   * zero. Gear numbers can't be repeated.
   */
  public final @NotNull Map<Integer, Shiftable.PerGearSettings> perGearSettings;
  /** The gear to start in. Can be null to use startingGearNum instead. */
  public final Shiftable.Gear startingGear;
  /**
   * The number of the gear to start in. Ignored if startingGear isn't null. Defaults to the lowest
   * gear.
   */
  public final Integer startingGearNum;
  /** The {@link com.revrobotics.CANSparkMax}s that are slaved to this controller. */
  public final @NotNull List<SlaveSparkMax> slaveSparks;
  /**
   * The gear settings at the start, based on {@link SmartMotorConfig#startingGear} or {@link
   * SmartMotorConfig#startingGearNum}
   */
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
