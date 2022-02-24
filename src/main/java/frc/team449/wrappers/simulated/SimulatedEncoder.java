package frc.team449.wrappers.simulated;

import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.team449.wrappers.Encoder;

public class SimulatedEncoder extends Encoder {
  private final EncoderSim wrapped;

  public SimulatedEncoder(
      edu.wpi.first.wpilibj.Encoder encoder,
      int encoderCPR,
      double unitPerRotation,
      double postEncoderGearing) {
    super("simulated_enc", encoderCPR, unitPerRotation, postEncoderGearing, false);
    this.wrapped = new EncoderSim(encoder);
  }

  @Override
  public void resetPosition() {
    wrapped.setReset(true);
  }

  @Override
  public double getPositionNative() {
    return wrapped.getDistance();
  }

  @Override
  public double getVelocityNative() {
    return 0;
  }

  @Override
  public double nativeToRPS(double nat) {
    return 0;
  }

  @Override
  public double rpsToNative(double rps) {
    return 0;
  }
}
