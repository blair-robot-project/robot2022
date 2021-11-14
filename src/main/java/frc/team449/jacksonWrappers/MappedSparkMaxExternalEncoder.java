package frc.team449.jacksonWrappers;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.team449.generalInterfaces.SmartMotorExternalEncoder;
import frc.team449.generalInterfaces.shiftable.Shiftable;
import frc.team449.other.Clock;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import java.util.Map;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.Nullable;

/** Represents a spark max with an external encoder */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class MappedSparkMaxExternalEncoder extends MappedSparkMaxBase
    implements SmartMotorExternalEncoder {
  /** The counts per rotation of the encoder being used, or null if there is no encoder. */
  @Nullable private final Integer encoderCPR;
  /** WPI provided encoder object */
  private final Encoder encoder;
  /** WPI provided PID Controller */
  private final PIDController pidController;

  private double lastTimeUpdate;

  @Log private double timeDiff;

  /**
   * Create a new SPARK MAX Controller
   *
   * @param port CAN port of this Spark.
   * @param name The Spark's name, used for logging purposes. Defaults to "spark_&gt;port&lt;"
   * @param reverseOutput Whether to reverse the output.
   * @param enableBrakeMode Whether to brake or coast when stopped.
   * @param PDP The PDP this Spark is connected to.
   * @param fwdLimitSwitchNormallyOpen Whether the forward limit switch is normally open or closed.
   *     If this is null, the forward limit switch is disabled.
   * @param revLimitSwitchNormallyOpen Whether the reverse limit switch is normally open or closed.
   *     If this is null, the reverse limit switch is disabled.
   * @param remoteLimitSwitchID The CAN ID the limit switch to use for this Spark is plugged into,
   *     or null to not use a limit switch.
   * @param fwdSoftLimit The forward software limit, in meters. If this is null, the forward
   *     software limit is disabled. Ignored if there's no encoder.
   * @param revSoftLimit The reverse software limit, in meters. If this is null, the reverse
   *     software limit is disabled. Ignored if there's no encoder.
   * @param postEncoderGearing The coefficient the output changes by after being measured by the
   *     encoder, e.g. this would be 1/70 if there was a 70:1 gearing between the encoder and the
   *     final output. Defaults to 1.
   * @param unitPerRotation The number of meters travelled per rotation of the motor this is
   *     attached to. Defaults to 1.
   * @param currentLimit The max amps this device can draw. If this is null, no current limit is
   *     used.
   * @param enableVoltageComp Whether or not to use voltage compensation. Defaults to false.
   * @param perGearSettings The settings for each gear this motor has. Can be null to use default
   *     values and gear # of zero. Gear numbers can't be repeated.
   * @param startingGear The gear to start in. Can be null to use startingGearNum instead.
   * @param startingGearNum The number of the gear to start in. Ignored if startingGear isn't null.
   *     Defaults to the lowest gear.
   * @param statusFrameRatesMillis The update rates, in millis, for each of the status frames.
   * @param controlFrameRateMillis The update rate, in milliseconds, for each control frame.
   */
  @JsonCreator
  public MappedSparkMaxExternalEncoder(
      @JsonProperty(required = true) final int port,
      @Nullable final String name,
      final boolean reverseOutput,
      @JsonProperty(required = true) final boolean enableBrakeMode,
      @Nullable final PDP PDP,
      @Nullable final Boolean fwdLimitSwitchNormallyOpen,
      @Nullable final Boolean revLimitSwitchNormallyOpen,
      @Nullable final Integer remoteLimitSwitchID,
      @Nullable final Double fwdSoftLimit,
      @Nullable final Double revSoftLimit,
      @Nullable final MappedDigitalInput encoderDIO1,
      @Nullable final MappedDigitalInput encoderDIO2,
      @Nullable final Double postEncoderGearing,
      @Nullable final Double unitPerRotation,
      @Nullable Integer encoderCPR,
      final boolean reverseSensor,
      @Nullable final Integer currentLimit,
      final boolean enableVoltageComp,
      @Nullable final List<PerGearSettings> perGearSettings,
      @Nullable final Shiftable.Gear startingGear,
      @Nullable final Integer startingGearNum,
      @Nullable final Map<CANSparkMax.PeriodicFrame, Integer> statusFrameRatesMillis,
      @Nullable final Integer controlFrameRateMillis,
      @Nullable final List<SlaveSparkMax> slaveSparks) {
    super(
        port,
        name,
        reverseOutput,
        enableBrakeMode,
        PDP,
        fwdLimitSwitchNormallyOpen,
        revLimitSwitchNormallyOpen,
        remoteLimitSwitchID,
        fwdSoftLimit,
        revSoftLimit,
        postEncoderGearing,
        unitPerRotation,
        currentLimit,
        enableVoltageComp,
        perGearSettings,
        startingGear,
        startingGearNum,
        statusFrameRatesMillis,
        controlFrameRateMillis,
        slaveSparks);

    if (encoderDIO1 != null && encoderDIO2 != null) {
      encoder = new Encoder(encoderDIO1, encoderDIO2, reverseSensor, CounterBase.EncodingType.k4X);
    } else {
      encoder = new Encoder(0, 1);
    }
    this.pidController = new PIDController(0, 0, 0);

    // todo determine if encoderCPR will ever be needed
    encoderCPR = encoderCPR != null ? encoderCPR : 1;
    this.encoderCPR = encoderCPR;

    encoder.setDistancePerPulse(1. / encoderCPR);
    encoder.setSamplesToAverage(5);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    this.pidController.setP(kP);
    this.pidController.setI(kI);
    this.pidController.setD(kD);
  }

  /**
   * Convert from native velocity units to output rotations per second. Note this DOES NOT account
   * for post-encoder gearing.
   *
   * @param nat A velocity in RPM
   * @return That velocity in RPS
   */
  @Contract(pure = true)
  @Override
  public Double nativeToRPS(final double nat) {
    return nat;
  }

  /**
   * Convert from output RPS to native velocity units. Note this DOES NOT account for post-encoder
   * gearing.
   *
   * @param rps The RPS velocity you want to convert.
   * @return That velocity in RPM
   */
  @Contract(pure = true)
  @Override
  public double rpsToNative(final double rps) {
    return rps;
  }

  @Override
  public void setVoltage(final double volts) {
    timeDiff -= lastTimeUpdate;
    lastTimeUpdate = Clock.currentTimeSeconds();
    spark.setVoltage(volts);
  }

  /**
   * Set a position setpoint for the Spark.
   *
   * @param meters An absolute position setpoint, in meters.
   */
  @Override
  public void setPositionSetpoint(final double meters) {
    this.setpoint = meters;
    double nativeSetpoint = this.unitToEncoder(meters);
    setVoltage(
        currentGearSettings.feedForwardCalculator.ks
            + pidController.calculate(encoderPosition(), nativeSetpoint));
  }

  /**
   * Give a velocity closed loop setpoint in MPS.
   *
   * @param velocity velocity setpoint in MPS.
   */
  @Override
  public void setVelocityUPS(final double velocity) {
    this.currentControlMode = ControlType.kVelocity;
    double nativeSetpoint = upsToEncoder(velocity);
    this.setpoint = velocity;
    setVoltage(
        currentGearSettings.feedForwardCalculator.calculate(velocity)
            + pidController.calculate(encoderVelocity(), nativeSetpoint));
  }

  @Override
  public Encoder getEncoder() {
    return this.encoder;
  }
}
