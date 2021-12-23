package frc.team449.jacksonWrappers;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.team449.generalInterfaces.MotorContainer;
import frc.team449.generalInterfaces.SmartMotor;
import frc.team449.javaMaps.builders.SmartMotorConfig;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Map;

public abstract class MappedSparkMaxBase implements SmartMotor, AutoCloseable {
  /** A list of all the gears this robot has and their settings. */
  @NotNull protected final Map<Integer, PerGearSettings> perGearSettings;
  /** REV brushless controller object */
  protected final CANSparkMax spark;
  /**
   * The number of meters travelled per rotation of the motor this is attached to, or null if there
   * is no encoder.
   */
  private final double unitPerRotation;
  /** Forward limit switch object */
  private final CANDigitalInput forwardLimitSwitch;
  /** Reverse limit switch object */
  private final CANDigitalInput reverseLimitSwitch;
  /** The Spark's name, used for logging purposes. */
  @NotNull private final String name;
  /** The settings currently being used by this Spark. */
  @NotNull protected PerGearSettings currentGearSettings;
  /** The control mode of the motor */
  protected ControlType currentControlMode;
  /** The most recently set setpoint. */
  protected double setpoint;
  /**
   * The coefficient the output changes by after being measured by the encoder, e.g. this would be
   * 1/70 if there was a 70:1 gearing between the encoder and the final output.
   */
  @Log private double postEncoderGearing;

  /**
   * Create a new SPARK MAX Controller
   *
   * @param controlFrameRateMillis The update rate, in milliseconds, for each control frame.
   * @param statusFrameRatesMillis The update rates, in millis, for each of the status frames.
   * @param cfg The configuration for this Spark
   */
  public MappedSparkMaxBase(
      @Nullable Integer controlFrameRateMillis,
      @Nullable final Map<CANSparkMax.PeriodicFrame, Integer> statusFrameRatesMillis,
      @NotNull SmartMotorConfig cfg) {
    this.spark = new CANSparkMax(cfg.getPort(), CANSparkMaxLowLevel.MotorType.kBrushless);
    this.spark.restoreFactoryDefaults();

    // Set the name to the given one or to spark_<portnum>
    this.name = cfg.getName() != null ? cfg.getName() : ("spark_" + cfg.getPort());
    // Set this to false because we only use reverseOutput for slaves.
    this.spark.setInverted(cfg.isReverseOutput());
    // Set brake mode
    this.spark.setIdleMode(
        cfg.isEnableBrakeMode() ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);

    // Set frame rates
    if (controlFrameRateMillis != null) {
      // Must be between 1 and 100 ms.
      this.spark.setControlFramePeriodMs(controlFrameRateMillis);
    }

    if (statusFrameRatesMillis != null) {
      for (var frame : statusFrameRatesMillis.keySet()) {
        this.spark.setPeriodicFramePeriod(frame, statusFrameRatesMillis.get(frame));
      }
    }

    this.unitPerRotation = cfg.getUnitPerRotation();

    // Initialize
    this.perGearSettings = cfg.getPerGearSettingsMap();
    this.currentGearSettings = cfg.getInitialGearSettings();

    this.postEncoderGearing = cfg.getPostEncoderGearing();

    // Only enable the limit switches if it was specified if they're normally open or closed.
    if (cfg.getFwdLimitSwitchNormallyOpen() != null) {
      if (cfg.getRemoteLimitSwitchID() != null) {
        // set CANDigitalInput to other limit switch
        System.out.println("Forwardlimitswitchnotnull");
        // todo why is this creating a new sparkmax?
        try (var remote =
            new CANSparkMax(
                cfg.getRemoteLimitSwitchID(), CANSparkMaxLowLevel.MotorType.kBrushless)) {
          this.forwardLimitSwitch =
              remote.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);
        }
      } else {
        this.forwardLimitSwitch =
            this.spark.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
      }
    } else {
      this.forwardLimitSwitch =
          this.spark.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
      this.forwardLimitSwitch.enableLimitSwitch(false);
    }
    if (cfg.getRevLimitSwitchNormallyOpen() != null) {
      if (cfg.getRemoteLimitSwitchID() != null) {
        System.out.println("Reverselimitswitchnotnull");
        // todo why is this creating a new sparkmax?
        try (var remote =
            new CANSparkMax(
                cfg.getRemoteLimitSwitchID(), CANSparkMaxLowLevel.MotorType.kBrushless)) {
          this.reverseLimitSwitch =
              remote.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);
        }
      } else {
        this.reverseLimitSwitch =
            this.spark.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);
      }
    } else {
      this.reverseLimitSwitch =
          this.spark.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
      this.reverseLimitSwitch.enableLimitSwitch(false);
    }

    if (cfg.getFwdSoftLimit() != null) {
      this.spark.setSoftLimit(
          CANSparkMax.SoftLimitDirection.kForward, cfg.getFwdSoftLimit().floatValue());
    }
    if (cfg.getRevSoftLimit() != null) {
      this.spark.setSoftLimit(
          CANSparkMax.SoftLimitDirection.kReverse, cfg.getRevSoftLimit().floatValue());
    }

    // Set the current limit if it was given
    if (cfg.getCurrentLimit() != null) {
      this.spark.setSmartCurrentLimit(cfg.getCurrentLimit());
    }

    if (cfg.isEnableVoltageComp()) {
      this.spark.enableVoltageCompensation(12);
    } else {
      this.spark.disableVoltageCompensation();
    }

    for (final SlaveSparkMax slave : cfg.getSlaveSparks()) {
      slave.setMasterSpark(this.spark, cfg.isEnableBrakeMode());
    }

    this.spark.burnFlash();

    MotorContainer.register(this);
  }

  @Override
  public void disable() {
    this.spark.disable();
  }

  @Override
  public void setPercentVoltage(double percentVoltage) {
    this.currentControlMode = ControlType.kVoltage;

    if (Math.abs(percentVoltage) <= 1.0) {
      this.setpoint = percentVoltage;
    } else {
      // Warn the user if they're setting Vbus to a number that's outside the range of values.
      Shuffleboard.addEventMarker(
          "WARNING: YOU ARE CLIPPING MAX PERCENT VBUS AT " + percentVoltage,
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
      // Set setpoint to -1.0 or 1.0 instead
      this.setpoint = Math.signum(percentVoltage);
    }

    this.spark.set(this.setpoint);
  }

  @Override
  @Log
  public int getGear() {
    return this.currentGearSettings.gear;
  }

  @Override
  public void setGear(final int gear) {
    // Set the current gear
    this.currentGearSettings = this.perGearSettings.get(gear);

    // note, no current limiting

    if (this.currentGearSettings.rampRate != null) {
      // Set ramp rate, converting from volts/sec to seconds until 12 volts.
      this.spark.setClosedLoopRampRate(1 / (this.currentGearSettings.rampRate / 12.));
      this.spark.setOpenLoopRampRate(1 / (this.currentGearSettings.rampRate / 12.));
    } else {
      this.spark.setClosedLoopRampRate(0);
      this.spark.setOpenLoopRampRate(0);
    }

    if (this.currentGearSettings.postEncoderGearing != null) {
      this.postEncoderGearing = currentGearSettings.postEncoderGearing;
    }

    this.setPID(
        this.currentGearSettings.kP, this.currentGearSettings.kI, this.currentGearSettings.kD);
  }

  /**
   * Convert from native units read by an encoder to meters moved. Note this DOES account for
   * post-encoder gearing.
   *
   * @param revs revolutions measured by the encoder
   * @return That distance in meters, or null if no encoder CPR was given.
   */
  @Override
  public double encoderToUnit(final double revs) {
    return revs * unitPerRotation * postEncoderGearing;
  }

  /**
   * Convert a distance from meters to encoder reading in native units. Note this DOES account for
   * post-encoder gearing.
   *
   * @param meters A distance in meters.
   * @return That distance in native units as measured by the encoder, or null if no encoder CPR was
   *     given.
   */
  @Override
  public double unitToEncoder(final double meters) {
    return meters / unitPerRotation / postEncoderGearing;
  }

  /**
   * Converts the velocity read by the getVelocity() method to the MPS of the output shaft. Note
   * this DOES account for post-encoder gearing.
   *
   * @param encoderReading The velocity read from the encoder with no conversions.
   * @return The velocity of the output shaft, in MPS, when the encoder has that reading, or null if
   *     no encoder CPR was given.
   */
  @Override
  public double encoderToUPS(final double encoderReading) {
    return nativeToRPS(encoderReading) * postEncoderGearing * unitPerRotation;
  }

  /**
   * Converts from the velocity of the output shaft to what the getVelocity() method would read at
   * that velocity. Note this DOES account for post-encoder gearing.
   *
   * @param MPS The velocity of the output shaft, in MPS.
   * @return What the raw encoder reading would be at that velocity, or null if no encoder CPR was
   *     given.
   */
  @Override
  public double upsToEncoder(final double MPS) {
    return rpsToNative((MPS / postEncoderGearing) / unitPerRotation);
  }

  @Override
  public void setVoltage(final double volts) {
    spark.setVoltage(volts);
  }

  /**
   * Set the velocity for the motor to go at.
   *
   * @param velocity the desired velocity, on [-1, 1].
   */
  @Override
  public void setVelocity(final double velocity) {
    if (currentGearSettings.maxSpeed != null) {
      setVelocityUPS(velocity * currentGearSettings.maxSpeed);
    } else {
      this.setPercentVoltage(velocity);
    }
  }

  @Override
  @Log
  public double getSetpoint() {
    return this.setpoint;
  }

  @Override
  @Log
  public double getOutputVoltage() {
    return this.spark.getAppliedOutput() * this.spark.getBusVoltage();
  }

  @Override
  @Log
  public double getBatteryVoltage() {
    return this.spark.getBusVoltage();
  }

  @Override
  @Log
  public double getOutputCurrent() {
    return this.spark.getOutputCurrent();
  }

  @Override
  public String getControlMode() {
    return this.currentControlMode.name();
  }

  @Override
  public void setGearScaledVelocity(final double velocity, final int gear) {
    if (currentGearSettings.maxSpeed != null) {
      setVelocityUPS(currentGearSettings.maxSpeed * velocity);
    } else {
      this.setPercentVoltage(velocity);
    }
  }

  @Override
  public void setGearScaledVelocity(final double velocity, final Gear gear) {
    this.setGearScaledVelocity(velocity, gear.getNumVal());
  }

  @Override
  public SimpleMotorFeedforward getCurrentGearFeedForward() {
    return this.currentGearSettings.feedForwardCalculator;
  }

  @Override
  public boolean isFwdLimitSwitch() {
    return this.forwardLimitSwitch.get();
  }

  @Override
  public boolean isRevLimitSwitch() {
    return this.reverseLimitSwitch.get();
  }

  @Override
  public boolean isInhibitedForward() {
    return this.spark.getFault(CANSparkMax.FaultID.kHardLimitFwd);
  }

  @Override
  public boolean isInhibitedReverse() {
    return this.spark.getFault(CANSparkMax.FaultID.kHardLimitRev);
  }

  @Override
  public int getPort() {
    return this.spark.getDeviceId();
  }

  @Override
  public void close() {
    this.spark.close();
  }

  @Override
  public String configureLogName() {
    return this.name;
  }
}
