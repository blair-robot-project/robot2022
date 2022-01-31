package frc.team449.javaMaps.builders;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.hal.util.HalHandleException;
import frc.team449.jacksonWrappers.WrappedEncoder;
import frc.team449.jacksonWrappers.WrappedMotor;
import org.jetbrains.annotations.Nullable;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/** Motor controller configuration, along with some Spark-specific stuff */
public final class SparkMaxConfig extends MotorConfig<SparkMaxConfig> {
  private final Map<CANSparkMax.PeriodicFrame, Integer> statusFrameRatesMillis = new HashMap<>();
  private @Nullable Integer controlFrameRateMillis;

  @Nullable
  public Integer getControlFrameRateMillis() {
    return this.controlFrameRateMillis;
  }

  public SparkMaxConfig setControlFrameRateMillis(int controlFrameRateMillis) {
    this.controlFrameRateMillis = controlFrameRateMillis;
    return this;
  }

  public Map<CANSparkMax.PeriodicFrame, Integer> getStatusFrameRatesMillis() {
    return new HashMap<>(this.statusFrameRatesMillis);
  }

  public SparkMaxConfig addStatusFrameRateMillis(
      CANSparkMaxLowLevel.PeriodicFrame frame, int rate) {
    this.statusFrameRatesMillis.put(frame, rate);
    return this;
  }

  public SparkMaxConfig copy() {
    var copy = new SparkMaxConfig();
    this.copyTo(copy);

    if (this.controlFrameRateMillis != null) {
      copy.setControlFrameRateMillis(this.controlFrameRateMillis);
    }

    copy.statusFrameRatesMillis.putAll(this.getStatusFrameRatesMillis());

    return copy;
  }

  @Override
  public WrappedMotor createReal() {
    var motor = new CANSparkMax(this.getPort(), CANSparkMaxLowLevel.MotorType.kBrushless);
    if (motor.getLastError() == REVLibError.kHALError) {
      throw new HalHandleException("Motor could not be constructed on port " + this.getPort());
    }
    var externalEncoder = this.getExternalEncoder();
    var encoderName =
        this.getName() != null ? this.getName() + "_enc" : "spark_enc_" + this.getPort();
    var wrappedEnc =
        externalEncoder == null
            ? new WrappedEncoder.SparkEncoder(
                encoderName,
                motor.getEncoder(),
                this.getUnitPerRotation(),
                this.getPostEncoderGearing())
            : new WrappedEncoder.WPIEncoder(
                encoderName,
                externalEncoder,
                this.getEncoderCPR(),
                this.getUnitPerRotation(),
                this.getPostEncoderGearing());

    motor.restoreFactoryDefaults();

    // todo Set this to false because we only use reverseOutput for slaves.
    motor.setInverted(this.isReverseOutput());
    // Set brake mode
    motor.setIdleMode(
        this.isEnableBrakeMode() ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);

    // Set frame rates
    if (this.getControlFrameRateMillis() != null) {
      // Must be between 1 and 100 ms.
      motor.setControlFramePeriodMs(this.getControlFrameRateMillis());
    }

    this.getStatusFrameRatesMillis().forEach(motor::setPeriodicFramePeriod);

    // todo handle limit switches
    if (this.getFwdLimitSwitchNormallyOpen() == null) {
      motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
    }
    if (this.getRevLimitSwitchNormallyOpen() == null) {
      motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
    }

    if (this.getFwdSoftLimit() != null) {
      motor.setSoftLimit(
          CANSparkMax.SoftLimitDirection.kForward, this.getFwdSoftLimit().floatValue());
    }
    if (this.getRevSoftLimit() != null) {
      motor.setSoftLimit(
          CANSparkMax.SoftLimitDirection.kReverse, this.getRevSoftLimit().floatValue());
    }

    // Set the current limit if it was given
    if (this.getCurrentLimit() != null) {
      motor.setSmartCurrentLimit(this.getCurrentLimit());
    }

    if (this.isEnableVoltageComp()) {
      motor.enableVoltageCompensation(12);
    } else {
      motor.disableVoltageCompensation();
    }

    for (var slave : this.getSlaveSparks()) {
      slave.setMasterSpark(motor, this.isEnableBrakeMode());
    }

    if (this.getRampRate() != null) {
      // Set ramp rate, converting from volts/sec to seconds until 12 volts.
      motor.setClosedLoopRampRate(12.0 / this.getRampRate());
      motor.setOpenLoopRampRate(12.0 / this.getRampRate());
    } else {
      motor.setClosedLoopRampRate(0);
      motor.setOpenLoopRampRate(0);
    }

    motor.burnFlash();

    return new WrappedMotor(
        Objects.requireNonNullElse(this.getName(), "spark_" + this.getPort()), motor, wrappedEnc);
  }
}
