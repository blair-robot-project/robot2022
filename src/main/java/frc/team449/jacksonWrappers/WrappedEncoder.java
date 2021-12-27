package frc.team449.jacksonWrappers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.Encoder;
import org.jetbrains.annotations.NotNull;

public abstract class WrappedEncoder {
  /** Counts per rotation of the encoder */
  private final int encoderCPR;
  /** Meters traveled per rotation of the motor */
  private final double unitPerRotation;

  /** @param unitPerRotation Meters traveled per rotation of the motor */
  public WrappedEncoder(int encoderCPR, double unitPerRotation) {
    this.unitPerRotation = unitPerRotation;
    this.encoderCPR = encoderCPR;
  }

  /** Reset position to 0 */
  public abstract void resetPosition();

  /**
   * Current position in encoder's units
   *
   * @see WrappedEncoder#getPositionUnits(double)
   */
  public abstract double getPosition();

  /**
   * Current velocity in encoder's units
   *
   * @see WrappedEncoder#getVelocityUnits(double)
   */
  public abstract double getVelocity();

  /**
   * Convert encoder units to our units (meters)
   *
   * @param revs Encoder measurement
   * @param postEncoderGearing The coefficient the output changes by after being measured by the
   *     encoder, e.g. this would be 1/70 if there was a 70:1 gearing between the encoder and the
   *     final output.
   * @return The encoder distance converted to meters
   */
  public final double encoderToUnit(double revs, double postEncoderGearing) {
    return revs * unitPerRotation * postEncoderGearing / encoderCPR;
  }

  /**
   * Convert meters to native encoder units
   *
   * @param meters Distance in meters
   * @param postEncoderGearing The coefficient the output changes by after being measured by the
   *     encoder, e.g. this would be 1/70 if there was a 70:1 gearing between the encoder and the
   *     final output.
   * @return A distance in encoder units
   */
  public final double unitToEncoder(double meters, double postEncoderGearing) {
    return meters * encoderCPR / unitPerRotation / postEncoderGearing;
  }

  /** Current position in meters */
  public final double getPositionUnits(double postEncoderGearing) {
    return this.encoderToUnit(this.getPosition(), postEncoderGearing);
  }

  /** Current velocity in meters */
  public final double getVelocityUnits(double postEncoderGearing) {
    return this.encoderToUnit(this.getVelocity(), postEncoderGearing);
  }

  public static class WPIEncoder extends WrappedEncoder {
    private final Encoder encoder;

    public WPIEncoder(@NotNull Encoder encoder, int encoderCPR, double unitPerRotation) {
      super(1, unitPerRotation);
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
  }

  public static class SparkEncoder extends WrappedEncoder {
    private final CANEncoder encoder;

    public SparkEncoder(@NotNull CANEncoder encoder, double unitPerRotation) {
      super(1, unitPerRotation);
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
  }

  public static class TalonEncoder extends WrappedEncoder {
    private final TalonSRX talon;

    public TalonEncoder(@NotNull TalonSRX talon, int encoderCPR, double unitPerRotation) {
      super(encoderCPR, unitPerRotation);
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
  }
}
