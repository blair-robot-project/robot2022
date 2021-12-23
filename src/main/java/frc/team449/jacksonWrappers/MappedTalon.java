package frc.team449.jacksonWrappers;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.team449.generalInterfaces.MotorContainer;
import frc.team449.generalInterfaces.SmartMotor;
import frc.team449.javaMaps.builders.SmartMotorConfig;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.List;
import java.util.Map;

/**
 * Component wrapper on the CTRE {@link TalonSRX}, with unit conversions to/from MPS built in. Every
 * non-unit-conversion in this class takes arguments in post-gearing MPS.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class MappedTalon implements SmartMotor {

  /** The CTRE CAN Talon SRX that this class is a wrapper on */
  @NotNull protected final TalonSRX canTalon;
  /** The counts per rotation of the encoder being used, or null if there is no encoder. */
  @Nullable private final Integer encoderCPR;
  /**
   * The number of meters travelled per rotation of the motor this is attached to, or null if there
   * is no encoder.
   */
  private final double unitPerRotation;
  /** A list of all the gears this robot has and their settings. */
  @NotNull private final Map<Integer, PerGearSettings> perGearSettings;
  /** The talon's name, used for logging purposes. */
  @NotNull private final String name;
  /** Whether the forwards or reverse limit switches are normally open or closed, respectively. */
  private final boolean fwdLimitSwitchNormallyOpen, revLimitSwitchNormallyOpen;
  /** The settings currently being used by this Talon. */
  @NotNull protected PerGearSettings currentGearSettings;

  final Faults faults = new Faults();
  /**
   * The coefficient the output changes by after being measured by the encoder, e.g. this would be
   * 1/70 if there was a 70:1 gearing between the encoder and the final output.
   */
  private double postEncoderGearing;
  /** The most recently set setpoint. */
  private double setpoint;

  private boolean voltageCompEnabled;

  /**
   * Default constructor.
   *
   * @param controlFrameRatesMillis The update rate, in milliseconds, for each of the control frame.
   * @param voltageCompSamples TALON-SPECIFIC. The number of 1-millisecond samples to use for
   *     voltage compensation. Defaults to 32.
   * @param feedbackDevice TALON-SPECIFIC. The type of encoder used to measure the output velocity
   *     of this motor. Can be null if there is no encoder attached to this controller.
   * @param encoderCPR TALON-SPECIFIC. The counts per rotation of the encoder on this controller.
   *     Can be null if feedbackDevice is, but otherwise must have a value.
   * @param reverseSensor TALON-SPECIFIC. Whether or not to reverse the reading from the encoder on
   *     this controller. Ignored if feedbackDevice is null. Defaults to false.
   * @param slaveTalons TALON-SPECIFIC. The {@link TalonSRX}s that are slaved to this controller.
   * @param slaveVictors TALON-SPECIFIC. The {@link com.ctre.phoenix.motorcontrol.can.VictorSPX}s
   * @param statusFrameRatesMillis The update rates, in millis, for each of the Talon status frames.
   * @param cfg The configuration for this Talon
   */
  @JsonCreator
  public MappedTalon(
      @Nullable final Map<ControlFrame, Integer> controlFrameRatesMillis,
      @Nullable final Integer voltageCompSamples,
      @Nullable final FeedbackDevice feedbackDevice,
      @Nullable final Integer encoderCPR,
      @Nullable final Boolean reverseSensor,
      @Nullable final List<SlaveTalon> slaveTalons,
      @Nullable final List<SlaveVictor> slaveVictors,
      @Nullable final Map<StatusFrameEnhanced, Integer> statusFrameRatesMillis,
      @NotNull final SmartMotorConfig cfg) {
    // Instantiate the base CANTalon this is a wrapper on.
    this.canTalon = new TalonSRX(cfg.getPort());
    // Set the name to the given one or to talon_portnum
    this.name = cfg.getName() != null ? cfg.getName() : ("talon_" + cfg.getPort());
    // Set this to false because we only use reverseOutput for slaves.
    this.canTalon.setInverted(cfg.isReverseOutput());
    // Set brake mode
    this.canTalon.setNeutralMode(cfg.isEnableBrakeMode() ? NeutralMode.Brake : NeutralMode.Coast);
    // Reset the position
    this.resetPosition();

    // Set frame rates
    if (controlFrameRatesMillis != null) {
      for (final ControlFrame controlFrame : controlFrameRatesMillis.keySet()) {
        this.canTalon.setControlFramePeriod(
            controlFrame, controlFrameRatesMillis.get(controlFrame));
      }
    }
    if (statusFrameRatesMillis != null) {
      for (final StatusFrameEnhanced statusFrame : statusFrameRatesMillis.keySet()) {
        this.canTalon.setStatusFramePeriod(statusFrame, statusFrameRatesMillis.get(statusFrame), 0);
      }
    }

    // Set fields
    this.unitPerRotation = cfg.getUnitPerRotation();

    // Initialize
    this.perGearSettings = cfg.getPerGearSettingsMap();
    this.currentGearSettings = cfg.getInitialGearSettings();

    // Only enable the limit switches if it was specified if they're normally open or closed.
    if (cfg.getFwdLimitSwitchNormallyOpen() != null) {
      if (cfg.getRemoteLimitSwitchID() != null) {
        this.canTalon.configForwardLimitSwitchSource(
            RemoteLimitSwitchSource.RemoteTalonSRX,
            cfg.getFwdLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            cfg.getRemoteLimitSwitchID(),
            0);
      } else {
        this.canTalon.configForwardLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector,
            cfg.getFwdLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            0);
      }
      this.fwdLimitSwitchNormallyOpen = cfg.getFwdLimitSwitchNormallyOpen();
    } else {
      this.canTalon.configForwardLimitSwitchSource(
          LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
      this.fwdLimitSwitchNormallyOpen = true;
    }
    if (cfg.getRevLimitSwitchNormallyOpen() != null) {
      if (cfg.getRemoteLimitSwitchID() != null) {
        this.canTalon.configReverseLimitSwitchSource(
            RemoteLimitSwitchSource.RemoteTalonSRX,
            cfg.getRevLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            cfg.getRemoteLimitSwitchID(),
            0);
      } else {
        this.canTalon.configReverseLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector,
            cfg.getRevLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            0);
      }
      this.revLimitSwitchNormallyOpen = cfg.getRevLimitSwitchNormallyOpen();
    } else {
      this.canTalon.configReverseLimitSwitchSource(
          LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
      this.revLimitSwitchNormallyOpen = true;
    }

    // Set up the feedback device if it exists.
    if (feedbackDevice != null) {
      // CTRE encoder use RPM instead of native units, and can be used as QuadEncoders, so we switch
      // them to avoid
      // having to support RPM.
      if (feedbackDevice.equals(FeedbackDevice.CTRE_MagEncoder_Absolute)
          || feedbackDevice.equals(FeedbackDevice.CTRE_MagEncoder_Relative)) {
        this.canTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
      } else {
        this.canTalon.configSelectedFeedbackSensor(feedbackDevice, 0, 0);
      }
      this.encoderCPR = encoderCPR;
      this.canTalon.setSensorPhase(reverseSensor != null && reverseSensor);

      // Only enable the software limits if they were given a value and there's an encoder.
      if (cfg.getFwdSoftLimit() != null) {
        this.canTalon.configForwardSoftLimitEnable(true, 0);
        this.canTalon.configForwardSoftLimitThreshold(
            (int) this.unitToEncoder(cfg.getFwdSoftLimit()), 0);
      } else {
        this.canTalon.configForwardSoftLimitEnable(false, 0);
      }
      if (cfg.getRevSoftLimit() != null) {
        this.canTalon.configReverseSoftLimitEnable(true, 0);
        this.canTalon.configReverseSoftLimitThreshold(
            (int) this.unitToEncoder(cfg.getRevSoftLimit()), 0);
      } else {
        this.canTalon.configReverseSoftLimitEnable(false, 0);
      }
    } else {
      this.encoderCPR = null;
      this.canTalon.configSelectedFeedbackSensor(FeedbackDevice.None, 0, 0);
    }

    // postEncoderGearing defaults to 1
    this.postEncoderGearing = cfg.getPostEncoderGearing();

    // Set up gear-based settings.
    this.setGear(currentGearSettings.gear);

    // Set the current limit if it was given
    if (cfg.getCurrentLimit() != null) {
      this.canTalon.configContinuousCurrentLimit(cfg.getCurrentLimit(), 0);
      this.canTalon.configPeakCurrentDuration(0, 0);
      this.canTalon.configPeakCurrentLimit(0, 0); // No duration
      this.canTalon.enableCurrentLimit(true);
    } else {
      // If we don't have a current limit, disable current limiting.
      this.canTalon.enableCurrentLimit(false);
    }

    // Enable or disable voltage comp
    if (cfg.isEnableVoltageComp()) {
      canTalon.enableVoltageCompensation(true);
      canTalon.configVoltageCompSaturation(12, 0);
      voltageCompEnabled = true;
    }

    final int notNullVoltageCompSamples = voltageCompSamples != null ? voltageCompSamples : 32;
    canTalon.configVoltageMeasurementFilter(notNullVoltageCompSamples, 0);

    // Use slot 0
    this.canTalon.selectProfileSlot(0, 0);

    if (slaveTalons != null) {
      // Set up slaves.
      for (final SlaveTalon slave : slaveTalons) {
        slave.setMaster(
            cfg.getPort(),
            cfg.isEnableBrakeMode(),
            cfg.getCurrentLimit(),
            cfg.isEnableVoltageComp() ? notNullVoltageCompSamples : null);
      }
    }

    if (slaveVictors != null) {
      // Set up slaves.
      for (final SlaveVictor slave : slaveVictors) {
        slave.setMaster(
            this.canTalon,
            cfg.isEnableBrakeMode(),
            cfg.isEnableVoltageComp() ? notNullVoltageCompSamples : null);
      }
    }

    for (final SlaveSparkMax slave : cfg.getSlaveSparks()) {
      slave.setMasterPhoenix(cfg.getPort(), cfg.isEnableBrakeMode());
    }

    canTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    canTalon.configVelocityMeasurementWindow(10);

    MotorContainer.register(this);
  }

  /** Disables the motor, if applicable. */
  @Override
  public void disable() {
    this.canTalon.set(ControlMode.Disabled, 0);
  }

  /**
   * Set the motor output voltage to a given percent of available voltage.
   *
   * @param percentVoltage percent of total voltage from [-1, 1]
   */
  @Override
  public void setPercentVoltage(double percentVoltage) {
    // Warn the user if they're setting Vbus to a number that's outside the range of values.
    if (Math.abs(percentVoltage) > 1.0) {
      Shuffleboard.addEventMarker(
          "WARNING: YOU ARE CLIPPING MAX PERCENT VBUS AT " + percentVoltage,
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
      percentVoltage = Math.signum(percentVoltage);
    }

    this.setpoint = percentVoltage;

    this.canTalon.set(ControlMode.PercentOutput, percentVoltage);
  }

  /** @return The gear this subsystem is currently in. */
  @Override
  @Log
  public int getGear() {
    return this.currentGearSettings.gear;
  }

  /**
   * Shift to a specific gear.
   *
   * @param gear Which gear to shift to.
   */
  @Override
  public void setGear(final int gear) {
    // Set the current gear
    this.currentGearSettings = this.perGearSettings.get(gear);

    if (currentGearSettings.postEncoderGearing != null) {
      this.postEncoderGearing = currentGearSettings.postEncoderGearing;
    }

    // Set max voltage
    this.canTalon.configPeakOutputForward(this.currentGearSettings.fwdPeakOutputVoltage / 12., 0);
    this.canTalon.configPeakOutputReverse(this.currentGearSettings.revPeakOutputVoltage / 12., 0);

    // Set min voltage
    this.canTalon.configNominalOutputForward(
        this.currentGearSettings.fwdNominalOutputVoltage / 12., 0);
    this.canTalon.configNominalOutputReverse(
        this.currentGearSettings.revNominalOutputVoltage / 12., 0);

    if (this.currentGearSettings.rampRate != null) {
      // Set ramp rate, converting from volts/sec to seconds until 12 volts.
      this.canTalon.configClosedloopRamp(1 / (this.currentGearSettings.rampRate / 12.), 0);
      this.canTalon.configOpenloopRamp(1 / (this.currentGearSettings.rampRate / 12.), 0);
    } else {
      this.canTalon.configClosedloopRamp(0, 0);
      this.canTalon.configOpenloopRamp(0, 0);
    }

    // Set PID stuff
    // Slot 0 velocity gains. We don't set F yet because that changes based on setpoint.
    this.canTalon.config_kP(0, this.currentGearSettings.kP, 0);
    this.canTalon.config_kI(0, this.currentGearSettings.kI, 0);
    this.canTalon.config_kD(0, this.currentGearSettings.kD, 0);
  }

  /**
   * Convert from native units read by an encoder to meters moved. Note this DOES account for
   * post-encoder gearing.
   *
   * @param nativeUnits A distance native units as measured by the encoder.
   * @return That distance in meters, or null if no encoder CPR was given.
   */
  @Override
  public double encoderToUnit(final double nativeUnits) {
    if (encoderCPR == null) {
      return Double.NaN;
    }
    return nativeUnits / (this.encoderCPR * 4) * this.postEncoderGearing * this.unitPerRotation;
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
    if (encoderCPR == null) {
      return Double.NaN;
    }
    return meters / this.unitPerRotation * (this.encoderCPR * 4) / this.postEncoderGearing;
  }

  /**
   * Converts the velocity read by the talon's getVelocity() method to the MPS of the output shaft.
   * Note this DOES account for post-encoder gearing.
   *
   * @param encoderReading The velocity read from the encoder with no conversions.
   * @return The velocity of the output shaft, in MPS, when the encoder has that reading, or null if
   *     no encoder CPR was given.
   */
  @Override
  public double encoderToUPS(final double encoderReading) {
    Double RPS = nativeToRPS(encoderReading);
    if (RPS == null) {
      return Double.NaN;
    }
    return RPS * this.postEncoderGearing * this.unitPerRotation;
  }

  /**
   * Converts from the velocity of the output shaft to what the talon's getVelocity() method would
   * read at that velocity. Note this DOES account for post-encoder gearing.
   *
   * @param UPS The velocity of the output shaft, in MPS.
   * @return What the raw encoder reading would be at that velocity, or null if no encoder CPR was
   *     given.
   */
  @Override
  public double upsToEncoder(final double UPS) {
    return rpsToNative((UPS / postEncoderGearing) / unitPerRotation);
  }

  /**
   * Convert from CANTalon native velocity units to output rotations per second. Note this DOES NOT
   * account for post-encoder gearing.
   *
   * @param nat A velocity in CANTalon native units.
   * @return That velocity in RPS, or null if no encoder CPR was given.
   */
  @Contract(pure = true)
  @Nullable
  @Override
  public Double nativeToRPS(final double nat) {
    if (this.encoderCPR == null) {
      return null;
    }
    return (nat / (this.encoderCPR * 4)) * 10; // 4 edges per count, and 10 100ms per second.
  }

  /**
   * Convert from output RPS to the CANTalon native velocity units. Note this DOES NOT account for
   * post-encoder gearing.
   *
   * @param rps The RPS velocity you want to convert.
   * @return That velocity in CANTalon native units, or null if no encoder CPR was given.
   */
  @Contract(pure = true)
  @Override
  public double rpsToNative(final double rps) {
    if (this.encoderCPR == null) {
      return Double.NaN;
    }
    return (rps / 10) * (this.encoderCPR * 4); // 4 edges per count, and 10 100ms per second.
  }

  /** @return Total ticks travelled for debug purposes */
  @Override
  public double encoderPosition() {
    return this.canTalon.getSelectedSensorPosition();
  }

  @Override
  public void setVoltage(final double volts) {
    if (voltageCompEnabled) {
      setPercentVoltage(volts / 12.);
    } else {
      setPercentVoltage(volts / getBatteryVoltage());
    }
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    this.canTalon.config_kP(0, kP, 0);
    this.canTalon.config_kI(0, kI, 0);
    this.canTalon.config_kD(0, kD, 0);
  }

  /**
   * Set a position setpoint for the Talon.
   *
   * @param meters An absolute position setpoint, in meters.
   */
  @Override
  public void setPositionSetpoint(final double meters) {
    this.setpoint = meters;
    double nativeSetpoint = this.unitToEncoder(meters);
    this.canTalon.config_kF(0, 0);
    this.canTalon.set(
        ControlMode.Position,
        nativeSetpoint,
        DemandType.ArbitraryFeedForward,
        this.currentGearSettings.feedForwardCalculator.ks / 12.);
  }

  /** @return Ticks per 100ms for debug purposes */
  @Override
  public double encoderVelocity() {
    return this.canTalon.getSelectedSensorVelocity();
  }

  /**
   * Get the velocity of the CANTalon in MPS.
   *
   * @return The CANTalon's velocity in MPS, or null if no encoder CPR was given.
   */
  @Override
  public double getVelocity() {
    return encoderToUPS(canTalon.getSelectedSensorVelocity(0));
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
      setPercentVoltage(velocity);
    }
  }

  /**
   * Give a velocity closed loop setpoint in MPS.
   *
   * @param velocity velocity setpoint in MPS.
   */
  @Override
  public void setVelocityUPS(final double velocity) {
    this.setpoint = velocity;
    double nativeSetpoint = upsToEncoder(velocity);
    this.canTalon.config_kF(0, 0, 0);
    this.canTalon.set(
        ControlMode.Velocity,
        nativeSetpoint,
        DemandType.ArbitraryFeedForward,
        currentGearSettings.feedForwardCalculator.calculate(velocity) / 12.);
  }

  /**
   * Get the current closed-loop velocity error in MPS. WARNING: will give garbage if not in
   * velocity mode.
   *
   * @return The closed-loop error in MPS, or null if no encoder CPR was given.
   */
  @Log
  @Override
  public double getVelocityError() {
    if (!canTalon.getControlMode().equals(ControlMode.Velocity)) {
      Shuffleboard.addEventMarker(
          "MappedTalon#getVelocityError",
          "Attempted to get velocity error when not in velocity mode",
          EventImportance.kTrivial);
    }

    return this.encoderToUPS(canTalon.getClosedLoopError(0));
  }

  /**
   * Get the current velocity setpoint of the Talon in MPS, the position setpoint in meters
   *
   * @return The setpoint in sensible units for the current control mode.
   */
  @Log
  @Override
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Get the voltage the Talon is currently drawing from the PDP.
   *
   * @return Voltage in volts.
   */
  @Log
  @Override
  public double getOutputVoltage() {
    return canTalon.getMotorOutputVoltage();
  }

  /**
   * Get the voltage available for the Talon.
   *
   * @return Voltage in volts.
   */
  @Log
  @Override
  public double getBatteryVoltage() {
    return canTalon.getBusVoltage();
  }

  /**
   * Get the current the Talon is currently drawing from the PDP.
   *
   * @return Current in amps.
   */
  @Log
  @Override
  public double getOutputCurrent() {
    return canTalon.getSupplyCurrent();
  }

  /**
   * Get the current control mode of the Talon. Please don't use this for anything other than
   * logging.
   *
   * @return Control mode as a string.
   */
  @Override
  public String getControlMode() {
    return this.canTalon.getControlMode().name();
  }

  /**
   * Set the velocity scaled to a given gear's max velocity. Used mostly when autoshifting.
   *
   * @param velocity The velocity to go at, from [-1, 1], where 1 is the max speed of the given
   *     gear.
   * @param gear The number of the gear to use the max speed from to scale the velocity.
   */
  @Override
  public void setGearScaledVelocity(final double velocity, final int gear) {
    if (this.currentGearSettings.maxSpeed != null) {
      this.setVelocityUPS(this.currentGearSettings.maxSpeed * velocity);
    } else {
      this.setPercentVoltage(velocity);
    }
  }

  /**
   * Set the velocity scaled to a given gear's max velocity. Used mostly when autoshifting.
   *
   * @param velocity The velocity to go at, from [-1, 1], where 1 is the max speed of the given
   *     gear.
   * @param gear The gear to use the max speed from to scale the velocity.
   */
  @Override
  public void setGearScaledVelocity(final double velocity, final Gear gear) {
    this.setGearScaledVelocity(velocity, gear.getNumVal());
  }

  /** @return Feedforward calculator for this gear */
  @Override
  public SimpleMotorFeedforward getCurrentGearFeedForward() {
    return this.currentGearSettings.feedForwardCalculator;
  }

  /** Resets the position of the Talon to 0. */
  @Override
  public void resetPosition() {
    canTalon.setSelectedSensorPosition(0, 0, 0);
  }

  /**
   * Get the status of the forwards limit switch.
   *
   * @return True if the forwards limit switch is closed, false if it's open or doesn't exist.
   */
  @Override
  public boolean isFwdLimitSwitch() {
    return fwdLimitSwitchNormallyOpen == canTalon.getSensorCollection().isFwdLimitSwitchClosed();
  }

  /**
   * Get the status of the reverse limit switch.
   *
   * @return True if the reverse limit switch is closed, false if it's open or doesn't exist.
   */
  @Override
  public boolean isRevLimitSwitch() {
    return this.revLimitSwitchNormallyOpen
        == this.canTalon.getSensorCollection().isRevLimitSwitchClosed();
  }

  @Override
  public boolean isInhibitedForward() {
    this.canTalon.getFaults(this.faults);
    return this.faults.ForwardLimitSwitch;
  }

  @Override
  public boolean isInhibitedReverse() {
    this.canTalon.getFaults(this.faults);
    return this.faults.ReverseLimitSwitch;
  }

  @Override
  public int getPort() {
    return this.canTalon.getDeviceID();
  }

  @Override
  public String configureLogName() {
    return this.name;
  }
}
