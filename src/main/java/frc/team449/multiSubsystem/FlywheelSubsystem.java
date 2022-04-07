package frc.team449.multiSubsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.motor.WrappedMotor;
import frc.team449.other.Clock;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import org.jetbrains.annotations.NotNull;

public class FlywheelSubsystem extends SubsystemBase implements Loggable {
  /** Whether or not to use the state space API for flywheels */
  private static final boolean USE_STATE_SPACE = false;

  @NotNull private final String name;

  @NotNull private final WrappedMotor motor;

  @NotNull private final LinearSystemLoop<N1, N1, N1> flywheelLoop;

  @NotNull private final SimpleMotorFeedforward feedforward;

  /** The time, in seconds, when the periodic method was last called */
  protected double lastTime = Double.NaN;

  /** The desired velocity of the flywheel */
  @Config private double targetVel;

  protected FlywheelSubsystem(
      @NotNull String name,
      @NotNull WrappedMotor motor,
      @NotNull LinearSystemLoop<N1, N1, N1> flywheelLoop,
      @NotNull SimpleMotorFeedforward feedforward) {
    this.name = name;
    this.motor = motor;
    this.flywheelLoop = flywheelLoop;
    this.feedforward = feedforward;
  }

  /**
   * @param encoderSim A simulated encoder, in case we're in simulation
   */
  @NotNull
  public static FlywheelSubsystem create(
      @NotNull String name,
      @NotNull WrappedMotor motor,
      @NotNull LinearSystemLoop<N1, N1, N1> flywheelLoop,
      @NotNull SimpleMotorFeedforward feedforward,
      @NotNull FlywheelSim sim,
      @NotNull EncoderSim encoderSim) {
    if (RobotBase.isReal()) {
      return new FlywheelSubsystem(name, motor, flywheelLoop, feedforward);
    } else {
      return new SimFlywheel(name, sim, encoderSim, motor, flywheelLoop, feedforward);
    }
  }

  public double getTargetVel() {
    return this.targetVel;
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
  protected final double nextVoltage() {
    var currTime = Timer.getFPGATimestamp();
    if (Double.isNaN(lastTime)) {
      this.lastTime = currTime - 0.02;
    }
    var dt = currTime - lastTime;
    this.lastTime = currTime;

    if (USE_STATE_SPACE) {
      flywheelLoop.setNextR(targetVel);
      flywheelLoop.correct(VecBuilder.fill(motor.getVelocity()));
      flywheelLoop.predict(dt);
      return flywheelLoop.getU(0) + feedforward.ks * Math.signum(targetVel);
    } else {
      return feedforward.calculate(targetVel);
    }
  }

  @Override
  public void periodic() {
    motor.setVoltage(this.nextVoltage());
  }

  @Override
  public String configureLogName() {
    return this.name;
  }

  public static final class SimFlywheel extends FlywheelSubsystem {
    @NotNull private final FlywheelSim sim;
    @NotNull private final EncoderSim encoderSim;
    /** Circumference of the motor, for converting to units per sec */
    private final double circumference;

    public SimFlywheel(
        @NotNull String name,
        @NotNull FlywheelSim sim,
        @NotNull EncoderSim encoderSim,
        @NotNull WrappedMotor motor,
        @NotNull LinearSystemLoop<N1, N1, N1> flywheelLoop,
        @NotNull SimpleMotorFeedforward feedforward) {
      super(name, motor, flywheelLoop, feedforward);
      this.sim = sim;
      this.encoderSim = encoderSim;
      this.circumference = motor.encoder.unitPerRotation;
      encoderSim.setInitialized(true);
      SmartDashboard.putData(
          "FlywheelSim" + name,
          builder -> {
            builder.addDoubleProperty("target", this::getTargetVel, this::setTargetVel);
            builder.addDoubleProperty("vel", encoderSim::getRate, encoderSim::setRate);
          });
    }

    @Override
    public void periodic() {
      sim.update(Double.isNaN(this.lastTime) ? 0.02 : Clock.currentTimeSeconds() - this.lastTime);
      sim.setInputVoltage(this.nextVoltage());
      encoderSim.setRate(sim.getAngularVelocityRadPerSec() * this.circumference);
    }
  }
}
