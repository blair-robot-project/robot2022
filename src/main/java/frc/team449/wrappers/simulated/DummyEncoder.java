package frc.team449.wrappers.simulated;

import frc.team449.wrappers.Encoder;

public class DummyEncoder extends Encoder {
  public DummyEncoder(int encoderCPR, double unitPerRotation, double postEncoderGearing) {
    super(encoderCPR, unitPerRotation, postEncoderGearing);
  }

  @Override
  public void resetPosition() {}

  @Override
  public double getPosition() {
    return 0;
  }

  @Override
  public double getVelocity() {
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
