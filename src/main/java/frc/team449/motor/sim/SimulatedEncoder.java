package frc.team449.motor.sim;

import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.team449.motor.Encoder;
import org.jetbrains.annotations.NotNull;

public final class SimulatedEncoder extends Encoder {
  public final @NotNull EncoderSim encSim;
  private double positionOffset = 0.0;

  public SimulatedEncoder(
      @NotNull String name,
      @NotNull EncoderSim encSim,
      int encoderCPR,
      double unitPerRotation,
      double postEncoderGearing) {
    super(name, encoderCPR, unitPerRotation, postEncoderGearing, false);
    this.encSim = encSim;
  }

  @Override
  public void resetPosition(double pos) {
    this.positionOffset = unitToEncoder(pos) - encSim.getDistance();
  }

  @Override
  public double getPositionNative() {
    return encSim.getDistance() + positionOffset;
  }

  @Override
  public double getVelocityNative() {
    return encSim.getRate();
  }

  @Override
  public double nativeToRPS(double nat) {
    return nat;
  }

  @Override
  public double rpsToNative(double rps) {
    return rps;
  }
}
