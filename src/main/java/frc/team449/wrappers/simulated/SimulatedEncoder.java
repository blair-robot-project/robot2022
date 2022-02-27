package frc.team449.wrappers.simulated;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.team449.wrappers.Encoder;
import org.jetbrains.annotations.NotNull;

public final class SimulatedEncoder extends Encoder {
  public final @NotNull EncoderSim encSim;

  public SimulatedEncoder(
      @NotNull String name,
      @NotNull EncoderSim encSim,
      int encoderCPR,
      double unitPerRotation,
      double postEncoderGearing) {
    super(name, encoderCPR, unitPerRotation, postEncoderGearing, false);
    this.encSim = encSim;
  }

  public SimulatedEncoder(
      @NotNull String name,
      @NotNull EncoderSim encSim,
      @NotNull DifferentialDrivetrainSim driveSim) {
    this(name, encSim, 1, 1, 1);
  }

  @Override
  public void resetPosition() {
    encSim.setReset(true);
  }

  @Override
  public double getPositionNative() {
    return encSim.getDistance();
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
