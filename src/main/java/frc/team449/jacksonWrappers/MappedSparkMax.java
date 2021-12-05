package frc.team449.jacksonWrappers;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import com.revrobotics.*;
import frc.team449.generalInterfaces.MotorContainer;
import frc.team449.generalInterfaces.SmartMotor;
import frc.team449.jacksonWrappers.simulated.MPSSmartMotorSimulated;
import frc.team449.javaMaps.builders.SmartMotorConfig;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Map;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class MappedSparkMax extends MappedSparkMaxBase implements SmartMotor {
  /** REV provided encoder object */
  private final CANEncoder canEncoder;
  /** REV provided PID Controller */
  private final CANPIDController pidController;

  /**
   * Create a new SPARK MAX Controller
   *
   * @param controlFrameRateMillis The update rate, in milliseconds, for each control frame.
   * @param statusFrameRatesMillis The update rates, in millis, for each of the status frames.
   * @param cfg The configuration for this Spark
   */
  @JsonCreator
  public MappedSparkMax(
      @Nullable final Integer controlFrameRateMillis,
      @Nullable final Map<CANSparkMax.PeriodicFrame, Integer> statusFrameRatesMillis,
      @NotNull final SmartMotorConfig cfg) {
    super(controlFrameRateMillis, statusFrameRatesMillis, cfg);
    this.canEncoder = this.spark.getEncoder();
    this.pidController = this.spark.getPIDController();
    this.resetPosition();
    MotorContainer.register(this);
  }

  /**
   * Tries to create a MappedSparkMax, but if there's a HAL error, it creates a {@link
   * frc.team449.jacksonWrappers.simulated.MPSSmartMotorSimulated} instead
   *
   * @see MappedSparkMax#MappedSparkMax(Integer, Map, SmartMotorConfig)
   */
  public static SmartMotor create(
      @Nullable final Integer controlFrameRateMillis,
      @Nullable final Map<CANSparkMax.PeriodicFrame, Integer> statusFrameRatesMillis,
      @NotNull final SmartMotorConfig cfg) {
    try (final var spark = new CANSparkMax(cfg.port, CANSparkMaxLowLevel.MotorType.kBrushless)) {
      spark.restoreFactoryDefaults();
      if (spark.getLastError() == CANError.kHALError) {
        System.out.println(
            "HAL error for spark on port "
                + cfg.port
                + "; assuming nonexistent and replacing with simulated controller");
        return new MPSSmartMotorSimulated(cfg);
      } else {
        return new MappedSparkMax(controlFrameRateMillis, statusFrameRatesMillis, cfg);
      }
    }
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    this.pidController.setP(kP, 0);
    this.pidController.setI(kI, 0);
    this.pidController.setD(kD, 0);
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
    return nat / 60.;
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
    return rps * 60.;
  }

  /** @return Total revolutions for debug purposes */
  @Override
  public double encoderPosition() {
    return this.canEncoder.getPosition();
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
    this.pidController.setFF(this.currentGearSettings.feedForwardCalculator.ks / 12.);
    this.pidController.setReference(
        nativeSetpoint,
        ControlType.kPosition,
        0,
        this.currentGearSettings.feedForwardCalculator.ks,
        CANPIDController.ArbFFUnits.kVoltage);
  }

  /** @return Current RPM for debug purposes */
  @Override
  @Log
  public double encoderVelocity() {
    return this.canEncoder.getVelocity();
  }

  /**
   * Get the velocity of the CANTalon in MPS.
   *
   * @return The CANTalon's velocity in MPS, or null if no encoder CPR was given.
   */
  @Override
  @Log
  public double getVelocity() {
    return this.encoderToUPS(canEncoder.getVelocity());
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
    System.out.println("Native: " + nativeSetpoint + ", orig vel: " + velocity);
    this.pidController.setFF(0);
    this.pidController.setReference(
        nativeSetpoint,
        ControlType.kVelocity,
        0,
        this.currentGearSettings.feedForwardCalculator.calculate(velocity),
        CANPIDController.ArbFFUnits.kVoltage);
  }

  @Log
  public double getPosition() {
    return canEncoder.getPosition();
  }

  @Override
  public double getPositionUnits() {
    return encoderToUnit(canEncoder.getPosition());
  }

  @Override
  public void resetPosition() {
    this.canEncoder.setPosition(0);
  }
}
