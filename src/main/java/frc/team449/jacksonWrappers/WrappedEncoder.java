package frc.team449.jacksonWrappers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Encoder;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public abstract class WrappedEncoder {
  /** Counts per rotation of the encoder */
  protected final int encoderCPR;
  /** Meters traveled per rotation of the motor */
  private final double unitPerRotation;
  /**
   * The coefficient the output changes by after being measured by the encoder, e.g. this would be
   * 1/70 if there was a 70:1 gearing between the encoder and the final output.
   */
  private final double postEncoderGearing;

  /**
   * @param encoderCPR Counts per rotation of the encoder
   * @param unitPerRotation Meters traveled per rotation of the motor
   * @param postEncoderGearing The factor the output changes by after being measured by the encoder
   */
  public WrappedEncoder(int encoderCPR, double unitPerRotation, double postEncoderGearing) {
    this.unitPerRotation = unitPerRotation;
    this.encoderCPR = encoderCPR;
    this.postEncoderGearing = postEncoderGearing;
  }

  /** Reset position to 0 */
  public abstract void resetPosition();

  /**
   * Current position in encoder's units
   *
   * @see WrappedEncoder#getPositionUnits()
   */
  @Log
  public abstract double getPosition();

  /**
   * Current velocity in encoder's units
   *
   * @see WrappedEncoder#getVelocityUnits()
   */
  @Log
  public abstract double getVelocity();

  /**
   * Convert encoder units to our units (meters)
   *
   * @param revs Encoder measurement
   * @return The encoder distance converted to meters
   */
  public final double encoderToUnit(double revs) {
    return revs * unitPerRotation * postEncoderGearing / encoderCPR;
  }

  /**
   * Convert meters to native encoder units
   *
   * @param meters Distance in meters
   * @return A distance in encoder units
   */
  public final double unitToEncoder(double meters) {
    return meters * encoderCPR / unitPerRotation / postEncoderGearing;
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
  public double upsToEncoder(final double MPS) {
    return rpsToNative((MPS / postEncoderGearing) / unitPerRotation);
  }

  /** Current position in meters */
  @Log
  public final double getPositionUnits() {
    return this.encoderToUnit(this.getPosition());
  }

  /** Current velocity in meters */
  public final double getVelocityUnits() {
    return this.encoderToUnit(this.getVelocity());
  }

  public static class WPIEncoder extends WrappedEncoder {
    private final Encoder encoder;

    public WPIEncoder(
        @NotNull Encoder encoder,
        int encoderCPR,
        double unitPerRotation,
        double postEncoderGearing) {
      super(1, unitPerRotation, postEncoderGearing);
      // Set field encoderCPR to 1 because the WPI encoder handles it itself
      encoder.setDistancePerPulse(1.0 / encoderCPR);
      encoder.setSamplesToAverage(5);
      this.encoder = encoder;
      this.resetPosition();
    }

    @Override
    public double getPosition() {
      return encoder.getDistance();
    }

    @Override
    public double getVelocity() {
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

  public static class SparkEncoder extends WrappedEncoder {
    private final RelativeEncoder encoder;

    public SparkEncoder(
        @NotNull RelativeEncoder encoder, double unitPerRotation, double postEncoderGearing) {
      super(1, unitPerRotation, postEncoderGearing);
      this.encoder = encoder;
      this.resetPosition();
    }

    @Override
    public double getPosition() {
      return encoder.getPosition();
    }

    @Override
    public double getVelocity() {
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

  public static class TalonEncoder extends WrappedEncoder {
    private final TalonSRX talon;

    public TalonEncoder(
        @NotNull TalonSRX talon,
        int encoderCPR,
        double unitPerRotation,
        double postEncoderGearing) {
      // The Talon multiplies its encoder count by 4
      super(encoderCPR * 4, unitPerRotation, postEncoderGearing);
      this.talon = talon;
      this.resetPosition();
    }

    @Override
    public double getPosition() {
      return this.talon.getSelectedSensorPosition(0);
    }

    @Override
    public double getVelocity() {
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
