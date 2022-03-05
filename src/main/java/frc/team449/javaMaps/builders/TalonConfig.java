package frc.team449.javaMaps.builders;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import frc.team449.other.FollowerUtils;
import frc.team449.wrappers.Encoder;
import frc.team449.wrappers.WrappedMotor;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.*;

/** Motor controller configuration, along with Talon-specific stuff */
public class TalonConfig extends MotorConfig<TalonConfig> {

  private final Map<ControlFrame, Integer> controlFrameRatesMillis = new HashMap<>();
  private final Map<StatusFrameEnhanced, Integer> statusFrameRatesMillis = new HashMap<>();
  private final List<TalonSRX> slaveTalons = new ArrayList<>();
  private final List<VictorSPX> slaveVictors = new ArrayList<>();
  private int voltageCompSamples = 32;
  private @Nullable FeedbackDevice feedbackDevice;
  private boolean reverseSensor = false;

  @NotNull
  public Map<ControlFrame, Integer> getControlFrameRatesMillis() {
    return new HashMap<>(this.controlFrameRatesMillis);
  }

  public TalonConfig addControlFrameRateMillis(@NotNull ControlFrame controlFrame, int updateRate) {
    this.controlFrameRatesMillis.put(controlFrame, updateRate);
    return this;
  }

  @NotNull
  public Map<StatusFrameEnhanced, Integer> getStatusFrameRatesMillis() {
    return new HashMap<>(this.statusFrameRatesMillis);
  }

  public TalonConfig addStatusFrameRateMillis(
      @NotNull StatusFrameEnhanced statusFrame, int updateRate) {
    this.statusFrameRatesMillis.put(statusFrame, updateRate);
    return this;
  }

  public int getVoltageCompSamples() {
    return this.voltageCompSamples;
  }

  public TalonConfig setVoltageCompSamples(int voltageCompSamples) {
    this.voltageCompSamples = voltageCompSamples;
    return this;
  }

  @Nullable
  public FeedbackDevice getFeedbackDevice() {
    return feedbackDevice;
  }

  public TalonConfig setFeedbackDevice(@NotNull FeedbackDevice feedbackDevice) {
    this.feedbackDevice = feedbackDevice;
    return this;
  }

  public boolean getReverseSensor() {
    return reverseSensor;
  }

  public TalonConfig setReverseSensor(boolean reverseSensor) {
    this.reverseSensor = reverseSensor;
    return this;
  }

  @NotNull
  public List<TalonSRX> getSlaveTalons() {
    return new ArrayList<>(slaveTalons);
  }

  public TalonConfig addSlaveTalon(@NotNull TalonSRX slaveTalon) {
    this.slaveTalons.add(slaveTalon);
    return this;
  }

  @NotNull
  public List<VictorSPX> getSlaveVictors() {
    return new ArrayList<>(slaveVictors);
  }

  public TalonConfig addSlaveVictor(@NotNull VictorSPX slaveVictor) {
    this.slaveVictors.add(slaveVictor);
    return this;
  }

  public TalonConfig copy() {
    var copy =
        new TalonConfig()
            .setReverseSensor(this.getReverseSensor())
            .setVoltageCompSamples(this.getVoltageCompSamples());
    if (this.getFeedbackDevice() != null) {
      copy.setFeedbackDevice(this.getFeedbackDevice());
    }
    this.copyTo(copy);

    copy.controlFrameRatesMillis.putAll(this.controlFrameRatesMillis);
    copy.statusFrameRatesMillis.putAll(this.statusFrameRatesMillis);
    copy.slaveTalons.addAll(this.getSlaveTalons());
    copy.slaveVictors.addAll(this.getSlaveVictors());

    return copy;
  }

  @Contract("-> new")
  @NotNull
  @Override
  public WrappedMotor createReal() {
    var motor = new WPI_TalonSRX(this.getPort());
    var externalEncoder = this.getExternalEncoder();
    var encoderName =
        this.getName() != null ? this.getName() + "_enc" : "talon_enc_" + this.getPort();
    var wrappedEnc =
        externalEncoder == null
            ? new Encoder.TalonEncoder(
                encoderName,
                motor,
                this.getEncoderCPR(),
                this.getUnitPerRotation(),
                this.getPostEncoderGearing(),
                this.getCalculateVel())
            : new Encoder.WPIEncoder(
                encoderName,
                externalEncoder,
                this.getEncoderCPR(),
                this.getUnitPerRotation(),
                this.getPostEncoderGearing(),
                this.getCalculateVel());

    motor.setInverted(this.isReverseOutput());
    // Set brake mode
    motor.setNeutralMode(this.isEnableBrakeMode() ? NeutralMode.Brake : NeutralMode.Coast);

    this.getControlFrameRatesMillis().forEach(motor::setControlFramePeriod);
    this.getStatusFrameRatesMillis().forEach(motor::setStatusFramePeriod);

    // Only enable the limit switches if it was specified if they're normally open or closed.
    if (this.getFwdLimitSwitchNormallyOpen() != null) {
      if (this.getRemoteLimitSwitchID() != null) {
        motor.configForwardLimitSwitchSource(
            RemoteLimitSwitchSource.RemoteTalonSRX,
            this.getFwdLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            this.getRemoteLimitSwitchID(),
            0);
      } else {
        motor.configForwardLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector,
            this.getFwdLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            0);
      }
    } else {
      motor.configForwardLimitSwitchSource(
          LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
    }
    if (this.getRevLimitSwitchNormallyOpen() != null) {
      if (this.getRemoteLimitSwitchID() != null) {
        motor.configReverseLimitSwitchSource(
            RemoteLimitSwitchSource.RemoteTalonSRX,
            this.getRevLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            this.getRemoteLimitSwitchID(),
            0);
      } else {
        motor.configReverseLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector,
            this.getRevLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            0);
      }
    } else {
      motor.configReverseLimitSwitchSource(
          LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
    }

    // Setup feedback device if it exists
    var feedbackDevice = this.getFeedbackDevice();
    if (feedbackDevice != null) {
      // CTRE encoder use RPM instead of native units, and can be used as QuadEncoders, so we switch
      // them to avoid
      // having to support RPM.
      if (feedbackDevice == FeedbackDevice.CTRE_MagEncoder_Absolute
          || feedbackDevice == FeedbackDevice.CTRE_MagEncoder_Relative) {
        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
      } else {
        motor.configSelectedFeedbackSensor(feedbackDevice, 0, 0);
      }
      motor.setSensorPhase(this.getReverseSensor());

      // Only enable the software limits if they were given a value and there's an encoder.
      if (this.getFwdSoftLimit() != null) {
        motor.configForwardSoftLimitEnable(true, 0);
        motor.configForwardSoftLimitThreshold(
            (int) wrappedEnc.unitToEncoder(this.getFwdSoftLimit()), 0);
      } else {
        motor.configForwardSoftLimitEnable(false, 0);
      }
      if (this.getRevSoftLimit() != null) {
        motor.configReverseSoftLimitEnable(true, 0);
        motor.configReverseSoftLimitThreshold(
            (int) wrappedEnc.unitToEncoder(this.getRevSoftLimit()), 0);
      } else {
        motor.configReverseSoftLimitEnable(false, 0);
      }
    } else {
      motor.configSelectedFeedbackSensor(FeedbackDevice.None, 0, 0);
    }

    // Set the current limit if it was given
    if (this.getCurrentLimit() != null) {
      motor.configContinuousCurrentLimit(this.getCurrentLimit(), 0);
      motor.configPeakCurrentDuration(0, 0);
      motor.configPeakCurrentLimit(0, 0); // No duration
      motor.enableCurrentLimit(true);
    } else {
      // If we don't have a current limit, disable current limiting.
      motor.enableCurrentLimit(false);
    }

    // Enable or disable voltage comp
    if (this.isEnableVoltageComp()) {
      motor.enableVoltageCompensation(true);
      motor.configVoltageCompSaturation(12, 0);
    }
    motor.configVoltageMeasurementFilter(this.getVoltageCompSamples(), 0);

    // Use slot 0
    motor.selectProfileSlot(0, 0);

    // Set up slaves.
    for (final TalonSRX slave : this.getSlaveTalons()) {
      FollowerUtils.setMasterForTalon(
          slave,
          this.getPort(),
          this.isEnableBrakeMode(),
          this.getCurrentLimit(),
          this.isEnableVoltageComp() ? this.getVoltageCompSamples() : null);
    }

    for (var slave : this.getSlaveVictors()) {
      FollowerUtils.setMasterForVictor(
          slave,
          motor,
          this.isEnableBrakeMode(),
          this.isEnableVoltageComp() ? this.getVoltageCompSamples() : null);
    }

    motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
    motor.configVelocityMeasurementWindow(10);

    // todo figure out how to get fwdPeakOutputVoltage and friends here
    // Set max voltage
    // motor.configPeakOutputForward(this.currentGearSettings.fwdPeakOutputVoltage / 12., 0);
    // motor.configPeakOutputReverse(this.currentGearSettings.revPeakOutputVoltage / 12., 0);
    motor.configPeakOutputForward(1.0, 0);
    motor.configPeakOutputReverse(1.0, 0);

    // Set min voltage
    // motor.configNominalOutputForward(this.currentGearSettings.fwdNominalOutputVoltage / 12., 0);
    // motor.configNominalOutputReverse(this.currentGearSettings.revNominalOutputVoltage / 12., 0);
    motor.configNominalOutputForward(0.0, 0);
    motor.configNominalOutputReverse(0.0, 0);

    if (this.getRampRate() != null) {
      // Set ramp rate, converting from volts/sec to seconds until 12 volts.
      motor.configClosedloopRamp(12.0 / this.getRampRate(), 0);
      motor.configOpenloopRamp(12.0 / this.getRampRate(), 0);
    } else {
      motor.configClosedloopRamp(0, 0);
      motor.configOpenloopRamp(0, 0);
    }

    return new WrappedMotor(
        Objects.requireNonNullElse(this.getName(), "talon_" + this.getPort()), motor, wrappedEnc);
  }
}
