package frc.team449.other;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.annotation.JsonCreator;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * Helps create {@link TalonSRX}'s that will be slaved to another TalonSRX or a {@link
 * com.ctre.phoenix.motorcontrol.can.VictorSPX}.
 */
public final class SlaveTalonUtils {

  private SlaveTalonUtils() {}

  /**
   * Default constructor.
   *
   * @param port The CAN ID of this Talon SRX.
   * @param invertType Whether or not to invert this Talon. Defaults to FollowMaster , but can be
   *     changed to OpposeMaster.
   */
  @JsonCreator
  public static TalonSRX createSlaveTalon(int port, InvertType invertType) {
    var talonSRX = new TalonSRX(port);

    // Turn off features we don't want a slave to have
    talonSRX.setInverted(invertType == null ? InvertType.FollowMaster : invertType);
    talonSRX.configForwardLimitSwitchSource(
        LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
    talonSRX.configReverseLimitSwitchSource(
        LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
    talonSRX.configForwardSoftLimitEnable(false, 0);
    talonSRX.configReverseSoftLimitEnable(false, 0);
    talonSRX.configPeakOutputForward(1, 0);
    talonSRX.enableVoltageCompensation(true);
    talonSRX.configVoltageCompSaturation(12, 0);
    talonSRX.configVoltageMeasurementFilter(32, 0);

    // Slow down frames so we don't overload the CAN bus
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 100, 0);

    return talonSRX;
  }

  /**
   * Set this Talon to follow another CAN device.
   *
   * @param port The CAN ID of the device to follow.
   * @param brakeMode Whether this Talon should be in brake mode or coast mode.
   * @param currentLimit The current limit for this Talon. Can be null for no current limit.
   * @param voltageCompSamples The number of voltage compensation samples to use, or null to not
   *     compensate voltage.
   */
  public static void setMaster(
      @NotNull TalonSRX talonSRX,
      final int port,
      final boolean brakeMode,
      @Nullable final Integer currentLimit,
      @Nullable final Integer voltageCompSamples) {
    // Brake mode doesn't automatically follow master
    talonSRX.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);

    // Current limiting might not automatically follow master, set it just to be safe
    if (currentLimit != null) {
      talonSRX.configContinuousCurrentLimit(currentLimit, 0);
      talonSRX.configPeakCurrentDuration(0, 0);
      talonSRX.configPeakCurrentLimit(0, 0); // No duration
      talonSRX.enableCurrentLimit(true);
    } else {
      // If we don't have a current limit, disable current limiting.
      talonSRX.enableCurrentLimit(false);
    }

    // Voltage comp might not follow master either
    if (voltageCompSamples != null) {
      talonSRX.enableVoltageCompensation(true);
      talonSRX.configVoltageCompSaturation(12, 0);
      talonSRX.configVoltageMeasurementFilter(voltageCompSamples, 0);
    } else {
      talonSRX.enableVoltageCompensation(false);
    }

    // Follow the leader
    talonSRX.set(ControlMode.Follower, port);
  }
}
