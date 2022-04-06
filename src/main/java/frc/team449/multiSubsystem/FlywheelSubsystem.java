package frc.team449.multiSubsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.motor.WrappedMotor;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class FlywheelSubsystem extends SubsystemBase implements Loggable {
  /** Whether or not to use the state space API for flywheel */
  private static final boolean USE_STATE_SPACE = true;

  @NotNull private final WrappedMotor motor;

  @NotNull private final LinearSystemLoop<N1, N1, N1> flywheelLoop;

  @NotNull private final SimpleMotorFeedforward feedforward;

  /** The time, in seconds, when the periodic method was last called */
  private double lastTime = Double.NaN;

  /** The desired velocity of the flywheel */
  @Log private double targetVel;

  protected FlywheelSubsystem(
      @NotNull WrappedMotor motor,
      @NotNull LinearSystemLoop<N1, N1, N1> flywheelLoop,
      @NotNull SimpleMotorFeedforward feedforward) {
    this.motor = motor;
    this.flywheelLoop = flywheelLoop;
    this.feedforward = feedforward;
  }

  /**
   * @param encoderSim A simulated encoder, in case we're in simulation
   */
  @NotNull
  public static FlywheelSubsystem create(
      @NotNull WrappedMotor motor,
      @NotNull LinearSystemLoop<N1, N1, N1> flywheelLoop,
      @NotNull SimpleMotorFeedforward feedforward,
      @NotNull FlywheelSim sim,
      @NotNull EncoderSim encoderSim) {
    if (RobotBase.isReal()) {
      return new FlywheelSubsystem(motor, flywheelLoop, feedforward);
    } else {
      return new SimFlywheel(sim, encoderSim, motor, flywheelLoop, feedforward);
    }
  }

  public void setTargetVel(double targetVel) {
    this.targetVel = targetVel;
  }

  /** Whether the flywheel's speed is within some tolerance of its target velocity */
  public boolean atSpeed(double tolerance) {
    return Math.abs(motor.getVelocity() - targetVel) < tolerance;
  }

  /** Get the flywheel's speed */
  public double getSpeed() {
    return motor.getVelocity();
  }

  public void stop() {
    setTargetVel(0);
  }

  /** Calculate the voltage to set for this iteration of the loop */
  protected double nextVoltage() {
    var currTime = Timer.getFPGATimestamp();
    if (Double.isNaN(lastTime)) {
      this.lastTime = currTime;
    }
    var dt = currTime - lastTime;
    this.lastTime = currTime;

    if (USE_STATE_SPACE) {
      flywheelLoop.setNextR(targetVel);
      flywheelLoop.correct(VecBuilder.fill(motor.getVelocity()));
      flywheelLoop.predict(dt);
      return flywheelLoop.getU(0);
    } else {
      return feedforward.calculate(targetVel);
    }
  }

  @Override
  public void periodic() {
    motor.setVoltage(this.nextVoltage());
  }

  public static final class SimFlywheel extends FlywheelSubsystem {
    @NotNull private final FlywheelSim sim;
    @NotNull private final EncoderSim encoderSim;
    /** Circumference of the motor, for converting to units per sec */
    private final double circumference;

    public SimFlywheel(
        @NotNull FlywheelSim sim,
        @NotNull EncoderSim encoderSim,
        @NotNull WrappedMotor motor,
        @NotNull LinearSystemLoop<N1, N1, N1> flywheelLoop,
        @NotNull SimpleMotorFeedforward feedforward) {
      super(motor, flywheelLoop, feedforward);
      this.sim = sim;
      this.encoderSim = encoderSim;
      this.circumference = motor.encoder.unitPerRotation;
    }

    @Override
    public void periodic() {
      sim.setInputVoltage(this.nextVoltage());
      encoderSim.setRate(sim.getAngularVelocityRadPerSec() * this.circumference);
    }
  }
}
