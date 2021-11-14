package frc.team449.jacksonWrappers;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.team449.generalInterfaces.SmartMotorExternalEncoder;
import frc.team449.javaMaps.builders.SmartMotorConfig;
import frc.team449.other.Clock;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Map;

/** Represents a spark max with an external encoder */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class MappedSparkMaxExternalEncoder extends MappedSparkMaxBase
    implements SmartMotorExternalEncoder {
  /** WPI provided encoder object */
  private final Encoder encoder;
  /** WPI provided PID Controller */
  private final PIDController pidController;

  private double lastTimeUpdate;

  @Log private double timeDiff;

  /**
   * Create a new SPARK MAX Controller
   *
   * @param encoderDIO1 The first encoder
   * @param encoderDIO2 The second encoder
   * @param statusFrameRatesMillis The update rates, in millis, for each of the status frames.
   * @param controlFrameRateMillis The update rate, in milliseconds, for each control frame.
   * @param cfg The configuration for this Spark
   */
  @JsonCreator
  public MappedSparkMaxExternalEncoder(
      @Nullable final MappedDigitalInput encoderDIO1,
      @Nullable final MappedDigitalInput encoderDIO2,
      @Nullable Integer encoderCPR,
      final boolean reverseSensor,
      @Nullable final Map<CANSparkMax.PeriodicFrame, Integer> statusFrameRatesMillis,
      @Nullable final Integer controlFrameRateMillis,
      @NotNull final SmartMotorConfig cfg) {
    super(controlFrameRateMillis, statusFrameRatesMillis, cfg);

    if (encoderDIO1 != null && encoderDIO2 != null) {
      encoder = new Encoder(encoderDIO1, encoderDIO2, reverseSensor, CounterBase.EncodingType.k4X);
    } else {
      encoder = new Encoder(0, 1);
    }
    this.pidController = new PIDController(0, 0, 0);

    encoder.setDistancePerPulse(encoderCPR != null ? 1. / encoderCPR : 1.);
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
