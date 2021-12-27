package frc.team449.jacksonWrappers;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.team449.generalInterfaces.MotorContainer;
import frc.team449.javaMaps.builders.SparkMaxConfig;
import frc.team449.javaMaps.builders.TalonConfig;
import io.github.oblarg.oblog.Loggable;
import java.util.Objects;
import org.jetbrains.annotations.NotNull;

public final class WrappedMotor implements SpeedController, Loggable {
  private final @NotNull SpeedController motor;
  public final @NotNull WrappedEncoder encoder;

  /** Name for logging */
  private final @NotNull String name;

  private WrappedMotor(
      @NotNull SpeedController motor, @NotNull WrappedEncoder encoder, @NotNull String name) {
    this.motor = motor;
    this.encoder = encoder;
    this.name = name;
    MotorContainer.register(this);
  }

  /**
   * Create a motor controller wrapping a Talon
   *
   * @param cfg Spark-specific configuration
   */
  public static WrappedMotor createSpark(SparkMaxConfig cfg) {
    var motor = new CANSparkMax(cfg.getPort(), CANSparkMaxLowLevel.MotorType.kBrushless);
    var externalEncoder = cfg.getExternalEncoder();
    var wrappedEnc =
        externalEncoder == null
            ? new WrappedEncoder.SparkEncoder(
                motor.getEncoder(), cfg.getUnitPerRotation(), cfg.getPostEncoderGearing())
            : new WrappedEncoder.WPIEncoder(
                externalEncoder,
                cfg.getEncoderCPR(),
                cfg.getUnitPerRotation(),
                cfg.getPostEncoderGearing());

    motor.restoreFactoryDefaults();

    // todo Set this to false because we only use reverseOutput for slaves.
    motor.setInverted(cfg.isReverseOutput());
    // Set brake mode
    motor.setIdleMode(
        cfg.isEnableBrakeMode() ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);

    // Set frame rates
    if (cfg.getControlFrameRateMillis() != null) {
      // Must be between 1 and 100 ms.
      motor.setControlFramePeriodMs(cfg.getControlFrameRateMillis());
    }

    cfg.getStatusFrameRatesMillis().forEach(motor::setPeriodicFramePeriod);

    // todo handle limit switches
    if (cfg.getFwdLimitSwitchNormallyOpen() == null) {
      motor
          .getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen)
          .enableLimitSwitch(false);
    }
    if (cfg.getRevLimitSwitchNormallyOpen() == null) {
      motor
          .getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen)
          .enableLimitSwitch(false);
    }

    if (cfg.getFwdSoftLimit() != null) {
      motor.setSoftLimit(
          CANSparkMax.SoftLimitDirection.kForward, cfg.getFwdSoftLimit().floatValue());
    }
    if (cfg.getRevSoftLimit() != null) {
      motor.setSoftLimit(
          CANSparkMax.SoftLimitDirection.kReverse, cfg.getRevSoftLimit().floatValue());
    }

    // Set the current limit if it was given
    if (cfg.getCurrentLimit() != null) {
      motor.setSmartCurrentLimit(cfg.getCurrentLimit());
    }

    if (cfg.isEnableVoltageComp()) {
      motor.enableVoltageCompensation(12);
    } else {
      motor.disableVoltageCompensation();
    }

    for (var slave : cfg.getSlaveSparks()) {
      slave.setMasterSpark(motor, cfg.isEnableBrakeMode());
    }

    if (cfg.getRampRate() != null) {
      // Set ramp rate, converting from volts/sec to seconds until 12 volts.
      motor.setClosedLoopRampRate(12.0 / cfg.getRampRate());
      motor.setOpenLoopRampRate(12.0 / cfg.getRampRate());
    } else {
      motor.setClosedLoopRampRate(0);
      motor.setOpenLoopRampRate(0);
    }

    motor.burnFlash();

    return new WrappedMotor(
        motor, wrappedEnc, Objects.requireNonNullElse(cfg.getName(), "spark_" + cfg.getPort()));
  }

  /**
   * Create a motor controller wrapping a Talon
   *
   * @param cfg Talon-specific configuration
   */
  public static WrappedMotor createTalon(TalonConfig cfg) {
    var motor = new WPI_TalonSRX(cfg.getPort());
    var externalEncoder = cfg.getExternalEncoder();
    var wrappedEnc =
        externalEncoder == null
            ? new WrappedEncoder.TalonEncoder(
                motor, cfg.getEncoderCPR(), cfg.getUnitPerRotation(), cfg.getPostEncoderGearing())
            : new WrappedEncoder.WPIEncoder(
                externalEncoder,
                cfg.getEncoderCPR(),
                cfg.getUnitPerRotation(),
                cfg.getPostEncoderGearing());

    // todo do only slaves need to be inverted?
    motor.setInverted(cfg.isReverseOutput());
    //Set brake mode
    motor.setNeutralMode(cfg.isEnableBrakeMode() ? NeutralMode.Brake : NeutralMode.Coast);

    cfg.getControlFrameRatesMillis().forEach(motor::setControlFramePeriod);
    cfg.getStatusFrameRatesMillis().forEach(motor::setStatusFramePeriod);

    // Only enable the limit switches if it was specified if they're normally open or closed.
    if (cfg.getFwdLimitSwitchNormallyOpen() != null) {
      if (cfg.getRemoteLimitSwitchID() != null) {
        motor.configForwardLimitSwitchSource(
            RemoteLimitSwitchSource.RemoteTalonSRX,
            cfg.getFwdLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            cfg.getRemoteLimitSwitchID(),
            0);
      } else {
        motor.configForwardLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector,
            cfg.getFwdLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            0);
      }
    } else {
      motor.configForwardLimitSwitchSource(
          LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
    }
    if (cfg.getRevLimitSwitchNormallyOpen() != null) {
      if (cfg.getRemoteLimitSwitchID() != null) {
        motor.configReverseLimitSwitchSource(
            RemoteLimitSwitchSource.RemoteTalonSRX,
            cfg.getRevLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            cfg.getRemoteLimitSwitchID(),
            0);
      } else {
        motor.configReverseLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector,
            cfg.getRevLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            0);
      }
    } else {
      motor.configReverseLimitSwitchSource(
          LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
    }

    // Setup feedback device if it exists
    var feedbackDevice = cfg.getFeedbackDevice();
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
      motor.setSensorPhase(cfg.getReverseSensor());

      // Only enable the software limits if they were given a value and there's an encoder.
      if (cfg.getFwdSoftLimit() != null) {
        motor.configForwardSoftLimitEnable(true, 0);
        motor.configForwardSoftLimitThreshold(
            (int) wrappedEnc.unitToEncoder(cfg.getFwdSoftLimit()), 0);
      } else {
        motor.configForwardSoftLimitEnable(false, 0);
      }
      if (cfg.getRevSoftLimit() != null) {
        motor.configReverseSoftLimitEnable(true, 0);
        motor.configReverseSoftLimitThreshold(
            (int) wrappedEnc.unitToEncoder(cfg.getRevSoftLimit()), 0);
      } else {
        motor.configReverseSoftLimitEnable(false, 0);
      }
    } else {
      motor.configSelectedFeedbackSensor(FeedbackDevice.None, 0, 0);
    }

    // Set the current limit if it was given
    if (cfg.getCurrentLimit() != null) {
      motor.configContinuousCurrentLimit(cfg.getCurrentLimit(), 0);
      motor.configPeakCurrentDuration(0, 0);
      motor.configPeakCurrentLimit(0, 0); // No duration
      motor.enableCurrentLimit(true);
    } else {
      // If we don't have a current limit, disable current limiting.
      motor.enableCurrentLimit(false);
    }

    // Enable or disable voltage comp
    if (cfg.isEnableVoltageComp()) {
      motor.enableVoltageCompensation(true);
      motor.configVoltageCompSaturation(12, 0);
    }
    motor.configVoltageMeasurementFilter(cfg.getVoltageCompSamples(), 0);

    // Use slot 0
    motor.selectProfileSlot(0, 0);

    // Set up slaves.
    for (final SlaveTalon slave : cfg.getSlaveTalons()) {
      slave.setMaster(
          cfg.getPort(),
          cfg.isEnableBrakeMode(),
          cfg.getCurrentLimit(),
          cfg.isEnableVoltageComp() ? cfg.getVoltageCompSamples() : null);
    }

    for (final SlaveVictor slave : cfg.getSlaveVictors()) {
      slave.setMaster(
          motor,
          cfg.isEnableBrakeMode(),
          cfg.isEnableVoltageComp() ? cfg.getVoltageCompSamples() : null);
    }

    for (final SlaveSparkMax slave : cfg.getSlaveSparks()) {
      slave.setMasterPhoenix(cfg.getPort(), cfg.isEnableBrakeMode());
    }

    motor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    motor.configVelocityMeasurementWindow(10);

    // todo figure out how to get fwdPeakOutputVoltage and friends here
    // Set max voltage
    //    motor.configPeakOutputForward(this.currentGearSettings.fwdPeakOutputVoltage / 12., 0);
    //    motor.configPeakOutputReverse(this.currentGearSettings.revPeakOutputVoltage / 12., 0);
    motor.configPeakOutputForward(1.0, 0);
    motor.configPeakOutputReverse(1.0, 0);

    // Set min voltage
    //    motor.configNominalOutputForward(this.currentGearSettings.fwdNominalOutputVoltage / 12.,
    // 0);
    //    motor.configNominalOutputReverse(this.currentGearSettings.revNominalOutputVoltage / 12.,
    // 0);
    motor.configNominalOutputForward(0.0, 0);
    motor.configNominalOutputReverse(0.0, 0);

    if (cfg.getRampRate() != null) {
      // Set ramp rate, converting from volts/sec to seconds until 12 volts.
      motor.configClosedloopRamp(12.0 / cfg.getRampRate(), 0);
      motor.configOpenloopRamp(12.0 / cfg.getRampRate(), 0);
    } else {
      motor.configClosedloopRamp(0, 0);
      motor.configOpenloopRamp(0, 0);
    }

    return new WrappedMotor(
        motor, wrappedEnc, Objects.requireNonNullElse(cfg.getName(), "talon_" + cfg.getPort()));
  }

  @Override
  public void set(double speed) {
    motor.set(speed);
  }

  @Override
  public void setVoltage(double outputVolts) {
    motor.setVoltage(outputVolts);
  }

  @Override
  public double get() {
    return motor.get();
  }

  @Override
  public void setInverted(boolean isInverted) {
    motor.setInverted(isInverted);
  }

  @Override
  public boolean getInverted() {
    return motor.getInverted();
  }

  @Override
  public void disable() {
    motor.disable();
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }

  @Override
  public void pidWrite(double output) {
    this.set(output);
  }

  @Override
  public String configureLogName() {
    return this.name;
  }
}
