package frc.team449.jacksonWrappers.simulated;

import frc.team449.jacksonWrappers.WrappedEncoder;

public class DummyWrappedEncoder extends WrappedEncoder {
  public DummyWrappedEncoder(int encoderCPR, double unitPerRotation, double postEncoderGearing) {
    super("dummy", encoderCPR, unitPerRotation, postEncoderGearing);
  }

  @Override
  public void resetPosition() {}

  @Override
  public double getPositionNative() {
    return 0;
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
