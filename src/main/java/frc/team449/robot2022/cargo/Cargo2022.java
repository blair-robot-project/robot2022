package frc.team449.robot2022.cargo;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.*;
import frc.team449.motor.WrappedMotor;
import frc.team449.other.Clock;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class Cargo2022 extends SubsystemBase implements Loggable {
  /** Whether or not to use the state space API for flywheel */
  private static final boolean USE_STATE_SPACE = true;
  /** The leader motor for the intake */
  private final WrappedMotor intakeMotor;
  /** The top motor that lets balls be spit out */
  private final WrappedMotor spitterMotor;
  /** State space model for shooter flywheel */
  private final @NotNull LinearSystemLoop<N1, N1, N1> flywheelLoop;
  /** Feed forward for the spitter */
  private final SimpleMotorFeedforward spitterFF;
  /** Motor used for shooting flywheel */
  private final WrappedMotor flywheelMotor;
  /** Feed forward for the flywheel */
  private final SimpleMotorFeedforward flywheelFF;
  /** Piston used to extend and retract intake */
  private final DoubleSolenoid deployIntake;
  /** Piston used to deploy and remove hood */
  private final DoubleSolenoid deployHood;
  /** Tracks the desired speed of the flywheel */
  @Log @Config private double flywheelSpeed = 0;
  /** Tracks the desired speed of the spitter */
  @Log @Config private double spitterSpeed = 0;
  /** Tracks if the balls are ready to shoot */
  private boolean isReady = false;
  /** The time, in seconds, when the periodic method was last called */
  private double lastTime = Double.NaN;

  public Cargo2022(
      @NotNull WrappedMotor intakeMotor,
      @NotNull WrappedMotor spitterMotor,
      @NotNull LinearSystemLoop<N1, N1, N1> flywheelLoop,
      @NotNull SimpleMotorFeedforward spitterFF,
      @NotNull WrappedMotor flywheelMotor,
      @NotNull SimpleMotorFeedforward flywheelFF,
      @NotNull DoubleSolenoid deployIntake,
      @NotNull DoubleSolenoid deployHood) {
    this.intakeMotor = intakeMotor;
    this.spitterMotor = spitterMotor;
    this.flywheelLoop = flywheelLoop;
    this.spitterFF = spitterFF;
    this.flywheelMotor = flywheelMotor;
    this.flywheelFF = flywheelFF;
    this.deployIntake = deployIntake;
    this.deployHood = deployHood;
  }

  public void runIntake() {
    intakeMotor.set(CargoConstants.FEEDER_OUTPUT);
    spitterSpeed = -CargoConstants.SPITTER_INTAKE_SPEED_RPS;
    isReady = false;
  }

  public void runIntakeReverse() {
    intakeMotor.set(-CargoConstants.FEEDER_OUTPUT);
    spitterSpeed = -CargoConstants.SPITTER_INTAKE_SPEED_RPS;
    isReady = true;
  }

  /**
   * Run the intake belts (but don't change the spitter and flywheel speeds) so the balls move up
   * and are ready to spit/shoot
   */
  private void startFeeding() {
    if (!hoodOn()) intakeMotor.set(CargoConstants.FEEDER_OUTPUT);
    else {
      intakeMotor.set(CargoConstants.INTAKE_SPEED_HIGH_SEQUENCE);
    }
  }

  /**
   * If the hood is up, spin up the spitter and flywheel. If the hood is down, only spin up the
   * spitter
   */
  private void spinUp() {
    if (hoodOn()) {
      spitterSpeed = CargoConstants.SPITTER_SHOOT_SPEED_RPS;
      flywheelSpeed = CargoConstants.SHOOTER_SPEED_RPS;
    } else {
      spitterSpeed = CargoConstants.SPITTER_SPIT_SPEED_RPS;
      flywheelSpeed = 0;
    }
  }

  public boolean atSpeed() {
    return Math.abs(flywheelSpeed - flywheelMotor.getVelocity()) < CargoConstants.SHOOTER_TOLERANCE
        && Math.abs(spitterSpeed - spitterMotor.getVelocity()) < CargoConstants.SHOOTER_TOLERANCE;
  }

  /** Prepares the shooter without actually shooting, minimizes shooting time */
  public Command ready() {
    return new InstantCommand(this::runIntakeReverse)
        .andThen(new WaitCommand(CargoConstants.REVERSE_BEFORE_SHOOT_TIME))
        .andThen(this::stop)
        .andThen(this::spinUp);
  }
  /**
   * Create a command that spins up the spitter (and flywheel, if the hood is on), waits till
   * they're at a speed to shoot, then spits/shoot
   */
  public Command startShooterCommand() {
    var preSpinUp =
        (isReady)
            ? new WaitCommand(0)
            : new InstantCommand(this::runIntakeReverse)
                .andThen(new WaitCommand(CargoConstants.REVERSE_BEFORE_SHOOT_TIME))
                .andThen(this::stop);
    var shootCommand =
        preSpinUp
            .andThen(this::stop)
            .andThen(this::spinUp)
            .andThen(
                new WaitCommand(CargoConstants.SHOOT_HIGH_SPINUP_TIME).withInterrupt(this::atSpeed))
            .andThen(this::startFeeding);
    var spitCommand = new InstantCommand(this::startFeeding).andThen(this::spinUp);
    return new ConditionalCommand(shootCommand, spitCommand, this::hoodOn);
  }

  /** Stop all motors */
  public void stop() {
    intakeMotor.set(0);
    spitterSpeed = 0;
    flywheelSpeed = 0;
  }

  public void stopFlywheel() {
    flywheelSpeed = 0;
  }

  public void deployIntake() {
    this.deployIntake.set(DoubleSolenoid.Value.kReverse);
  }

  public void retractIntake() {
    this.deployIntake.set(DoubleSolenoid.Value.kForward);
  }

  public void deployHood() {
    deployHood.set(DoubleSolenoid.Value.kReverse);
  }

  public void removeHood() {
    deployHood.set(DoubleSolenoid.Value.kForward);
  }

  /** Is the hood on to shoot or is it out of the way to spit? */
  private boolean hoodOn() {
    return deployHood.get() == DoubleSolenoid.Value.kReverse;
  }

  @Override
  public void periodic() {
    if (USE_STATE_SPACE) {
      var currTime = Clock.currentTimeSeconds();
      if (Double.isNaN(lastTime)) {
        this.lastTime = currTime;
      }
      var dt = currTime - lastTime;
      flywheelLoop.setNextR(flywheelSpeed);
      flywheelLoop.correct(VecBuilder.fill(flywheelMotor.getVelocity()));
      flywheelLoop.predict(dt);
      flywheelMotor.setVoltage(flywheelLoop.getU(0));
      this.lastTime = currTime;
    } else {
      //    double flywheelDelta = flywheelSpeed - flywheelMotor.getVelocity();
      //    double spitterDelta = spitterSpeed - spitterMotor.getVelocity();
      flywheelMotor.setVoltage(flywheelFF.calculate(flywheelSpeed /*, flywheelDelta / 0.02*/));
    }

    spitterMotor.setVoltage(spitterFF.calculate(spitterSpeed /*, spitterDelta / 0.02*/));
  }
}
