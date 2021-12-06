package frc.team449.jacksonWrappers.simulated;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.generalInterfaces.MotorContainer;
import frc.team449.generalInterfaces.SmartMotor;
import frc.team449.generalInterfaces.shiftable.Shiftable;
import frc.team449.generalInterfaces.updatable.Updatable;
import frc.team449.jacksonWrappers.SlaveTalon;
import frc.team449.jacksonWrappers.SlaveVictor;
import frc.team449.javaMaps.builders.SmartMotorConfig;
import frc.team449.other.Clock;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

import static frc.team449.other.Util.clamp;
import static frc.team449.other.Util.getLogPrefix;

/**
 * Class that implements {@link SmartMotor} without relying on the existence of actual hardware.
 * This class simulates a smart motor controller. Motor physics are simulated by {@link
 * SimulatedMotor}.
 *
 * <p>This class is automatically instantiated by the MPSSmartMotor factory method when the robot is
 * running in a simulation and should not be otherwise referenced in code.
 *
 * <p>The current implementation relies on fictional physics and does not involve
 */
public class MPSSmartMotorSimulated implements SmartMotor, Updatable {
  /** Maximum PID integral for anti-windup. */
  private static final double MAX_INTEGRAL = Double.POSITIVE_INFINITY;

  @NotNull private final String name;
  private final Type controllerType;
  private final int port;
  private final boolean reverseOutput;
  private final double unitPerRotation;
  private final boolean enableVoltageComp;
  @NotNull private final Map<Integer, Shiftable.PerGearSettings> perGearSettings;
  /** (V) */
  private final double busVoltage = SimulatedMotor.NOMINAL_VOLTAGE;
  /** (Depends on mode) */
  @Log private double setpoint;

  @NotNull
  private final MPSSmartMotorSimulated.PID pid =
      new PID(MAX_INTEGRAL, () -> this.setpoint, 0, 0, 0);

  @Log.ToString @NotNull private ControlMode controlMode = ControlMode.Disabled;
  @NotNull private Shiftable.PerGearSettings currentGearSettings;
  // Log the getters instead because logging the fields doesn't cause physics updates.
  private double percentOutput;

  @NotNull
  private final SimulatedMotor motor =
      new SimulatedMotor(() -> this.busVoltage * this.percentOutput);

  @Log private double lastStateUpdateTime = Clock.currentTimeMillis();

  public MPSSmartMotorSimulated(
      // Spark-specific
      @Nullable final Map<CANSparkMaxLowLevel.PeriodicFrame, Integer> sparkStatusFramesMap,
      @Nullable final Integer controlFrameRateMillis,
      // Talon-specific
      @Nullable final EnumMap<StatusFrameEnhanced, Integer> talonStatusFramesMap,
      @Nullable final Map<ControlFrame, Integer> controlFrameRatesMillis,
      @Nullable final RunningLinRegComponent voltagePerCurrentLinReg,
      @Nullable final Integer voltageCompSamples,
      @Nullable final FeedbackDevice feedbackDevice,
      @Nullable final Integer encoderCPR,
      @Nullable final Boolean reverseSensor,
      @Nullable final Double updaterProcessPeriodSecs,
      @Nullable final List<SlaveTalon> slaveTalons,
      @Nullable final List<SlaveVictor> slaveVictors,
      //General config
      @NotNull final SmartMotorConfig cfg
      ) {
    this.controllerType = cfg.getType();
    this.port = cfg.getPort();
    this.reverseOutput = cfg.isReverseOutput();
    this.unitPerRotation = cfg.getUnitPerRotation();
    this.enableVoltageComp = cfg.isEnableVoltageComp();
    this.name =
        cfg.getName() != null
            ? cfg.getName()
            : String.format(
                "%s_%d",
                cfg.getType() == Type.SPARK
                    ? "spark"
                    : cfg.getType() == Type.TALON ? "talon" : "MotorControllerUnknownType",
                port);

    this.perGearSettings = cfg.getPerGearSettingsMap();
    this.currentGearSettings = cfg.getInitialGearSettings();
    // Set up gear-based settings.
    this.setGear(currentGearSettings.gear);

    MotorContainer.register(this);
  }

  /**
   * Construct a simulated controller but without any Spark- or Talon-specific settings
   *
   * @param cfg The general configurations for the controller
   */
  public MPSSmartMotorSimulated(@NotNull final SmartMotorConfig cfg) {
    this(null, null, null, null, null, null, null, null, null, null, null, null, cfg);
  }

  public void setControlModeAndSetpoint(final ControlMode controlMode, final double setpoint) {
    this.controlMode = controlMode;
    this.setpoint = setpoint;

    switch (controlMode) {
      case Velocity:
        this.pid.reconfigure(
            this.currentGearSettings.kP, this.currentGearSettings.kI, this.currentGearSettings.kD);
        break;
      case Position:
        this.pid.reconfigure(
            this.currentGearSettings.posKP,
            this.currentGearSettings.posKI,
            this.currentGearSettings.posKD);
        break;

      case Current:
      case Follower:
        System.out.println("WARNING: Not yet implemented.");
        break;

      case MotionProfile:
      case MotionMagic:
      case MotionProfileArc:
        System.out.println("WARNING: Unlikely to be implemented.");
        break;

      case Disabled:
      case PercentOutput:
        // These modes require no additional action.
        break;
    }
  }

  /** Performs simulated PID logic and simulates physical state changes since last call. */
  private void updateSimulation() {
    final double now = Clock.currentTimeMillis();

    final double deltaMillis = (now - this.lastStateUpdateTime);
    final double deltaSecs = deltaMillis * 0.001;

    this.updateControllerLogic(deltaSecs);
    this.motor.updatePhysics(deltaSecs);

    this.lastStateUpdateTime = now;
  }

  private void updateControllerLogic(final double deltaSecs) {
    final double targetPercentOutput;
    switch (this.controlMode) {
      case PercentOutput:
        targetPercentOutput = this.setpoint;
        break;

      case Velocity:
      case Position:
        final double newActualValue =
            (this.controlMode == ControlMode.Velocity
                ? this.motor.getVelocity()
                : this.motor.getPosition());
        final double newError = this.setpoint - newActualValue;
        this.pid.update(this.reverseOutput ? -newError : newError, deltaSecs);
        targetPercentOutput = this.pid.getOutput();
        break;

      case Disabled:
        return;

      default:
        System.out.println(getLogPrefix(this) + "UNSUPPORTED CONTROL MODE " + this.controlMode);
        return;
    }

    final double actualTargetPercentOutput =
        this.reverseOutput ? -targetPercentOutput : targetPercentOutput;

    final double targetPercentOutputDelta = clamp(actualTargetPercentOutput - this.percentOutput);
    final double targetVoltageDelta = targetPercentOutputDelta * this.busVoltage;

    final double voltageDelta =
        this.currentGearSettings.rampRate == null
            ? targetVoltageDelta
            : clamp(targetVoltageDelta, this.currentGearSettings.rampRate * deltaSecs);
    final double percentOutputDelta = voltageDelta / this.busVoltage;

    this.percentOutput = clamp(this.percentOutput + percentOutputDelta);
  }

  /**
   * Set the motor output voltage to a given percent of available voltage.
   *
   * @param percentVoltage percent of total voltage from [-1, 1]
   */
  @Override
  public void setPercentVoltage(final double percentVoltage) {
    this.setControlModeAndSetpoint(ControlMode.PercentOutput, percentVoltage);
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
    return nativeUnits * this.unitPerRotation;
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
    return meters / this.unitPerRotation;
  }

  /**
   * Converts the velocity read by the controllers's getVelocity() method to the MPS of the output
   * shaft. Note this DOES account for post-encoder gearing.
   *
   * @param encoderReading The velocity read from the encoder with no conversions.
   * @return The velocity of the output shaft, in MPS, when the encoder has that reading, or null if
   *     no encoder CPR was given.
   */
  @Override
  public double encoderToUPS(final double encoderReading) {
    return encoderReading * this.unitPerRotation;
  }

  /**
   * Converts from the velocity of the output shaft to what the controllers's getVelocity() method
   * would read at that velocity. Note this DOES account for post-encoder gearing.
   *
   * @param MPS The velocity of the output shaft, in MPS.
   * @return What the raw encoder reading would be at that velocity, or null if no encoder CPR was
   *     given.
   */
  @Override
  public double upsToEncoder(final double MPS) {
    return MPS / this.unitPerRotation;
  }

  /**
   * Convert from native velocity units to output rotations per second. Note this DOES NOT account
   * for post-encoder gearing.
   *
   * @param nat A velocity in native units.
   * @return That velocity in RPS, or null if no encoder CPR was given.
   */
  @Override
  public Double nativeToRPS(final double nat) {
    return nat;
  }

  /**
   * Convert from output RPS to the native velocity. Note this DOES NOT account for post-encoder
   * gearing.
   *
   * @param rps The RPS velocity you want to convert.
   * @return That velocity in native units, or null if no encoder CPR was given.
   */
  @Override
  public double rpsToNative(final double rps) {
    return rps;
  }

  /** @return Raw position units for debugging purposes */
  @Override
  @Log
  public double encoderPosition() {
    return this.motor.getPosition();
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    // Nothing to do here
  }

  /** Set a position setpoint for the controller. */
  @Override
  public void setPositionSetpoint(final double meters) {
    this.setControlModeAndSetpoint(ControlMode.Position, this.unitToEncoder(meters));
  }

  /** @return Raw velocity units for debugging purposes */
  @Log
  @Override
  public double encoderVelocity() {
    return this.motor.getVelocity();
  }

  @Override
  public void setVoltage(final double volts) {
    this.setControlModeAndSetpoint(
        ControlMode.PercentOutput,
        this.enableVoltageComp
            ? volts / this.getBatteryVoltage()
            : volts / SimulatedMotor.NOMINAL_VOLTAGE);
  }

  /**
   * Get the velocity of the controller in MPS.
   *
   * @return The controller's velocity in MPS, or null if no encoder CPR was given.
   */
  @Log
  @Override
  public double getVelocity() {
    return this.encoderToUPS(this.encoderVelocity());
  }

  /**
   * Set the velocity for the motor to go at.
   *
   * @param velocity the desired velocity, on [-1, 1].
   */
  @Override
  public void setVelocity(final double velocity) {
    if (this.currentGearSettings.maxSpeed != null) {
      this.setVelocityUPS(velocity * this.currentGearSettings.maxSpeed);
    } else {
      this.setPercentVoltage(velocity);
    }
  }

  /** Enables the motor, if applicable. */
  @Override
  public void enable() {
    // Do nothing.
  }

  /** Disables the motor, if applicable. */
  @Override
  public void disable() {
    this.percentOutput = 0;
    this.setControlModeAndSetpoint(ControlMode.Disabled, 0);
  }

  /**
   * Give a velocity closed loop setpoint in MPS.
   *
   * @param velocity velocity setpoint in MPS.
   */
  @Override
  public void setVelocityUPS(final double velocity) {
    this.setControlModeAndSetpoint(ControlMode.Velocity, this.upsToEncoder(velocity));
  }

  /**
   * Get the current closed-loop velocity error in MPS. WARNING: will give garbage if not in
   * velocity mode.
   *
   * @return The closed-loop error in MPS, or null if no encoder CPR was given.
   */
  @Override
  public double getError() {
    return this.encoderToUPS(this.setpoint - this.motor.getVelocity());
  }

  /**
   * Get the current velocity setpoint of the Talon in MPS, the position setpoint in meters
   *
   * @return The setpoint in sensible units for the current control mode.
   */
  @Override
  public double getSetpoint() {
    switch (this.controlMode) {
      case Velocity:
        return this.encoderToUPS(this.setpoint);
      case Position:
        return this.encoderToUnit(this.setpoint);
      default:
        return Double.NaN;
    }
  }

  /**
   * Get the voltage the Talon is currently drawing from the PDP.
   *
   * @return Voltage in volts.
   */
  @Override
  @Log
  public double getOutputVoltage() {
    return this.getBatteryVoltage() * this.percentOutput;
  }

  /**
   * Get the voltage available for the Talon.
   *
   * @return Voltage in volts.
   */
  @Override
  @Log
  public double getBatteryVoltage() {
    return this.busVoltage;
  }

  /**
   * Get the current the Talon is currently drawing from the PDP.
   *
   * @return Current in amps.
   */
  @Override
  @Log
  public double getOutputCurrent() {
    return this.motor.getCurrent();
  }

  /**
   * Get the current control mode of the Talon. Please don't use this for anything other than
   * logging.
   *
   * @return Control mode as a string.
   */
  @Override
  public String getControlMode() {
    return this.controlMode.name();
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
  public void setGearScaledVelocity(final double velocity, final Shiftable.Gear gear) {
    this.setGearScaledVelocity(velocity, gear.getNumVal());
  }

  /** @return Feedforward calculator for this gear */
  @Override
  public SimpleMotorFeedforward getCurrentGearFeedForward() {
    return currentGearSettings.feedForwardCalculator;
  }

  /** @return the position of the talon in meters, or null of inches per rotation wasn't given. */
  @Override
  public double getPositionUnits() {
    return this.encoderToUnit(this.encoderPosition());
  }

  /** Resets the position of the Talon to 0. */
  @Override
  public void resetPosition() {
    this.motor.resetPosition();
    this.pid.resetState();
  }

  /**
   * Get the status of the forwards limit switch.
   *
   * @return True if the forwards limit switch is closed, false if it's open or doesn't exist.
   */
  @Override
  public boolean getFwdLimitSwitch() {
    return false;
  }

  /**
   * Get the status of the reverse limit switch.
   *
   * @return True if the reverse limit switch is closed, false if it's open or doesn't exist.
   */
  @Override
  public boolean getRevLimitSwitch() {
    return false;
  }

  @Override
  public boolean isInhibitedForward() {
    return false;
  }

  @Override
  public boolean isInhibitedReverse() {
    return false;
  }

  @Override
  public int getPort() {
    return this.port;
  }

  /** @return The gear this subsystem is currently in. */
  @Override
  public int getGear() {
    return 0;
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
  }

  @Override
  public String configureLogName() {
    return this.name;
  }

  @Override
  public boolean isSimulated() {
    return true;
  }

  /** Updates all cached values with current ones. */
  @Override
  public void update() {
    this.updateSimulation();
  }

  private static class PID implements Loggable {
    private final DoubleSupplier getSetPoint;
    private final double maxIntegral;
    @Log private double error, integral, derivative;
    private double kP;
    private double kI;
    private double kD;

    public PID(
        final double maxIntegral,
        final DoubleSupplier getSetPoint,
        final double kP,
        final double kI,
        final double kD) {
      this.maxIntegral = maxIntegral;
      this.getSetPoint = getSetPoint;
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
    }

    public void update(final double newError, final double deltaSecs) {
      this.integral += (this.error + newError) * 0.5 * deltaSecs;
      this.integral = clamp(this.integral, this.maxIntegral);
      this.derivative = (newError - this.error) / deltaSecs;
      this.error = newError;
    }

    @Log
    public double getOutput() {
      return this.kP * this.error + this.kI * this.integral + this.kD * this.derivative;
    }

    public void reconfigure(final double kP, final double kI, final double kD) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.resetState();
    }

    public void resetState() {
      this.error = this.derivative = this.integral = 0;
    }
  }
}
