package frc.team449.javaMaps.builders;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxLimitSwitch;
import frc.team449.other.FollowerUtils;
import frc.team449.wrappers.Encoder;
import frc.team449.wrappers.WrappedMotor;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/** Motor controller configuration, along with some Spark-specific stuff */
public final class SparkMaxConfig extends MotorConfig<SparkMaxConfig> {
  private final Map<CANSparkMax.PeriodicFrame, Integer> statusFrameRatesMillis = new HashMap<>();
  private final @NotNull Map<CANSparkMax, Boolean> slaveSparks = new HashMap<>();
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

  public @NotNull Map<CANSparkMax, Boolean> getSlaveSparks() {
    return slaveSparks;
  }

  /**
   * Add a slave spark
   *
   * @param inverted Whether or not it's inverted
   */
  public SparkMaxConfig addSlaveSpark(@NotNull CANSparkMax slaveSpark, boolean inverted) {
    this.slaveSparks.put(slaveSpark, inverted);
    return this;
  }

  public SparkMaxConfig copy() {
    var copy = new SparkMaxConfig();
    this.copyTo(copy);

    if (this.controlFrameRateMillis != null) {
      copy.setControlFrameRateMillis(this.controlFrameRateMillis);
    }

    copy.statusFrameRatesMillis.putAll(this.getStatusFrameRatesMillis());

    slaveSparks.forEach(copy::addSlaveSpark);

    return copy;
  }

  @Contract(" -> new")
  @NotNull
  @Override
  public WrappedMotor createReal() {
    var motor = new CANSparkMax(this.getPort(), CANSparkMaxLowLevel.MotorType.kBrushless);
    if (motor.getLastError() != REVLibError.kOk) {
      throw new Error(
          "Motor could not be constructed on port "
              + this.getPort()
              + " due to error "
              + motor.getLastError());
    }
    var externalEncoder = this.getExternalEncoder();
    var encoderName =
        this.getName() != null ? this.getName() + "_enc" : "spark_enc_" + this.getPort();
    var wrappedEnc =
        externalEncoder == null
            ? new Encoder.SparkEncoder(
                encoderName,
                motor.getEncoder(),
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
          CANSparkMax.SoftLimitDirection.kForward,
          (float) wrappedEnc.unitToEncoder(this.getFwdSoftLimit()));
    }
    if (this.getRevSoftLimit() != null) {
      motor.setSoftLimit(
          CANSparkMax.SoftLimitDirection.kReverse,
          (float) wrappedEnc.unitToEncoder(this.getRevSoftLimit()));
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

    var brakeMode =
        this.isEnableBrakeMode() ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast;
    this.slaveSparks.forEach(
        (slave, inverted) -> FollowerUtils.setMasterForSpark(slave, motor, brakeMode, inverted));

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
