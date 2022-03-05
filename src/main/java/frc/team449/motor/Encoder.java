package frc.team449.motor;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;
import frc.team449.other.Clock;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public abstract class Encoder implements Loggable {
  /** Counts per rotation of the encoder */
  protected final int encoderCPR;
  private final String name;
  /** Meters traveled per rotation of the motor */
  private final double unitPerRotation;
  /**
   * The coefficient the output changes by after being measured by the encoder, e.g. this would be
   * 1/70 if there was a 70:1 gearing between the encoder and the final output.
   */
  private final double postEncoderGearing;
  /**
   * Whether or not to calculate velocity on our own instead of using the encoder's implementation
   */
  private final boolean calculateVel;
  /** The last measured position, in case we're calculating velocity on our own */
  private double prevPos;
  /** The time at which the last position was measured */
  private double prevTime;

  /**
   * @param encoderCPR Counts per rotation of the encoder
   * @param unitPerRotation Meters traveled per rotation of the motor
   * @param postEncoderGearing The factor the output changes by after being measured by the encoder
   * @param calculateVel Whether or not we should calculate velocity ourselves (instead of using the
   *     encoder's own velocity)
   */
  public Encoder(
      @NotNull String name,
      int encoderCPR,
      double unitPerRotation,
      double postEncoderGearing,
      boolean calculateVel) {
    this.name = name;
    this.unitPerRotation = unitPerRotation;
    this.encoderCPR = encoderCPR;
    this.postEncoderGearing = postEncoderGearing;
    this.calculateVel = calculateVel;
  }

  /** Reset position to 0 */
  public abstract void resetPosition();

  /**
   * Current position in encoder's units
   *
   * @see Encoder#getPositionUnits()
   */
  @Log
  public abstract double getPositionNative();

  /**
   * Current velocity in encoder's units
   *
   * @see Encoder#getVelocityUnits()
   */
  @Log
  public abstract double getVelocityNative();

  /**
   * Convert encoder units to our units (meters)
   *
   * @param revs Encoder measurement
   * @return The encoder distance converted to meters
   */
  public final double encoderToUnit(double revs) {
    return revs * unitPerRotation / postEncoderGearing / encoderCPR;
  }

  /**
   * Convert meters to native encoder units
   *
   * @param meters Distance in meters
   * @return A distance in encoder units
   */
  public final double unitToEncoder(double meters) {
    return meters * encoderCPR * postEncoderGearing / unitPerRotation;
  }

  /**
   * Convert from native velocity units to output rotations per second. Note this DOES NOT account
   * for post-encoder gearing.
   *
   * @param nat A velocity in RPM
   * @return That velocity in RPS
   */
  public abstract double nativeToRPS(final double nat);

  /**
   * Convert from output RPS to native velocity units. Note this DOES NOT account for post-encoder
   * gearing.
   *
   * @param rps The RPS velocity you want to convert.
   * @return That velocity in RPM
   */
  public abstract double rpsToNative(final double rps);

  /**
   * Converts the velocity read by the getVelocity() method to the MPS of the output shaft. Note
   * this DOES account for post-encoder gearing.
   *
   * @param encoderReading The velocity read from the encoder with no conversions.
   * @return The velocity of the output shaft, in MPS, when the encoder has that reading, or null if
   *     no encoder CPR was given.
   */
  public double encoderToUPS(final double encoderReading) {
    return nativeToRPS(encoderReading) * unitPerRotation / postEncoderGearing;
  }

  /**
   * Converts from the velocity of the output shaft to what the getVelocity() method would read at
   * that velocity. Note this DOES account for post-encoder gearing.
   *
   * @param MPS The velocity of the output shaft, in MPS.
   * @return What the raw encoder reading would be at that velocity, or null if no encoder CPR was
   *     given.
   */
  public double upsToEncoder(final double MPS) {
    return rpsToNative((MPS * postEncoderGearing) / unitPerRotation);
  }

  /** Current position in meters */
  @Log
  public final double getPositionUnits() {
    return this.encoderToUnit(this.getPositionNative());
  }

  /** Current velocity in meters */
  @Log
  public final double getVelocityUnits() {
    if (this.calculateVel) {
      // If we're calculating velocity ourselves, return change in position / change in time
      var currPos = this.getPositionUnits();
      var currTime = Clock.currentTimeSeconds();
      var vel = (currPos - this.prevPos) / (currTime - prevTime);
      this.prevPos = currPos;
      this.prevTime = currTime;
      return vel;
    } else {
      return this.encoderToUnit(this.getVelocityNative());
    }
  }

  @Override
  public String configureLogName() {
    return this.name;
  }

  public static class WPIEncoder extends Encoder {
    private final edu.wpi.first.wpilibj.Encoder encoder;

    public WPIEncoder(
        @NotNull String name,
        @NotNull edu.wpi.first.wpilibj.Encoder encoder,
        int encoderCPR,
        double unitPerRotation,
        double postEncoderGearing,
        boolean calculateVel) {
      super(name, 1, 1, 1, calculateVel);
      // Let the WPI encoder handle the distance scaling
      encoder.setDistancePerPulse(unitPerRotation * postEncoderGearing / encoderCPR);
      encoder.setSamplesToAverage(5);
      this.encoder = encoder;
      this.resetPosition();
    }

    @Override
    public double getPositionNative() {
      return encoder.getDistance();
    }

    @Override
    public double getVelocityNative() {
      return encoder.getRate();
    }

    @Override
    public void resetPosition() {
      encoder.reset();
    }

    @Override
    public double nativeToRPS(final double nat) {
      return nat;
    }

    @Override
    public double rpsToNative(final double rps) {
      return rps;
    }
  }

  public static class SparkEncoder extends Encoder {
    private final RelativeEncoder encoder;

    /**
     * @param name Motor name used for logging
     * @param encoder Actual encoder to wrap
     * @param unitPerRotation Meters per rotation
     * @param postEncoderGearing Factor output is multiplied by after encoders. NOTE: This should be
     *     >1, not a reciprocal
     */
    public SparkEncoder(
        @NotNull String name,
        @NotNull RelativeEncoder encoder,
        double unitPerRotation,
        double postEncoderGearing,
        boolean calculateVel) {
      super(name, 1, unitPerRotation, postEncoderGearing, calculateVel);
      this.encoder = encoder;
      this.resetPosition();
    }

    @Override
    public double getPositionNative() {
      return encoder.getPosition();
    }

    @Override
    public double getVelocityNative() {
      return encoder.getVelocity();
    }

    @Override
    public void resetPosition() {
      encoder.setPosition(0);
    }

    @Override
    public double nativeToRPS(final double nat) {
      return nat / 60.;
    }

    @Override
    public double rpsToNative(final double rps) {
      return rps * 60.;
    }
  }

  public static class TalonEncoder extends Encoder {
    private final TalonSRX talon;

    public TalonEncoder(
        @NotNull String name,
        @NotNull TalonSRX talon,
        int encoderCPR,
        double unitPerRotation,
        double postEncoderGearing,
        boolean calculateVel) {
      // The Talon multiplies its encoder count by 4
      super(name, encoderCPR * 4, unitPerRotation, postEncoderGearing, calculateVel);
      this.talon = talon;
      this.resetPosition();
    }

    @Override
    public double getPositionNative() {
      return this.talon.getSelectedSensorPosition(0);
    }

    @Override
    public double getVelocityNative() {
      return this.talon.getSelectedSensorVelocity(0);
    }

    @Override
    public void resetPosition() {
      this.talon.setSelectedSensorPosition(0, 0, 0);
    }

    @Override
    public double nativeToRPS(final double nat) {
      // 10 100ms per second.
      return nat / this.encoderCPR * 10;
    }

    @Override
    public double rpsToNative(final double rps) {
      // 10 100ms per second.
      return rps / 10 * this.encoderCPR;
    }
  }
}
