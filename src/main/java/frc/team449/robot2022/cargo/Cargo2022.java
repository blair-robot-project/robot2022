package frc.team449.robot2022.cargo;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449.motor.WrappedMotor;
import frc.team449.multiSubsystem.FlywheelSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class Cargo2022 extends SubsystemBase implements Loggable {
  /** The leader motor for the intake */
  private final WrappedMotor intakeMotor;
  /** The top motor that lets balls be spit out */
  private final FlywheelSubsystem spitter;
  /** Motor used for shooting flywheel */
  private final FlywheelSubsystem shooter;
  /** Piston used to extend and retract intake */
  private final DoubleSolenoid deployIntake;
  /** Piston used to deploy and remove hood */
  private final DoubleSolenoid deployHood;
  /** Tracks if the balls are ready to shoot */
  @Log private boolean isReady = false;

  public Cargo2022(
      @NotNull WrappedMotor intakeMotor,
      @NotNull FlywheelSubsystem spitter,
      @NotNull FlywheelSubsystem shooter,
      @NotNull DoubleSolenoid deployIntake,
      @NotNull DoubleSolenoid deployHood) {
    this.intakeMotor = intakeMotor;
    this.spitter = spitter;
    this.shooter = shooter;
    this.deployIntake = deployIntake;
    this.deployHood = deployHood;
  }

  public void runIntake() {
    intakeMotor.set(CargoConstants.FEEDER_OUTPUT);
    spitter.setTargetVel(-CargoConstants.SPITTER_INTAKE_SPEED_RPS);
    this.isReady = false;
  }

  public void runIntakeReverse() {
    intakeMotor.set(-CargoConstants.FEEDER_OUTPUT);
    spitter.setTargetVel(-CargoConstants.SPITTER_INTAKE_SPEED_RPS);
    this.isReady = true;
  }

  /**
   * Run the intake belts (but don't change the spitter and shooter speeds) so the balls move up and
   * are ready to spit/shoot
   */
  private void startFeeding() {
    if (!hoodOn()) {
      intakeMotor.set(CargoConstants.FEEDER_OUTPUT);
    } else {
      intakeMotor.set(CargoConstants.INTAKE_SPEED_HIGH_SEQUENCE);
    }
  }

  /**
   * If the hood is up, spin up the spitter and shooter. If the hood is down, only spin up the
   * spitter
   */
  private void spinUp() {
    switch (deployHood.get()) {
      case kReverse:
        spitter.setTargetVel(CargoConstants.SPITTER_SHOOT_SPEED_RPS);
        shooter.setTargetVel(CargoConstants.SHOOTER_SPEED_RPS);
        break;
      case kForward:
        spitter.setTargetVel(CargoConstants.SPITTER_SPIT_SPEED_RPS);
        shooter.stop();
      case kOff:
        spitter.setTargetVel(CargoConstants.SPITTER_SHOOT_SPIT_SIDE_SPEED);
        shooter.setTargetVel(CargoConstants.SHOOTER_SHOOT_SPIT_SIDE_SPEED);
    }
  }

  public boolean atSpeed() {
    return shooter.atSpeed(CargoConstants.SHOOTER_TOLERANCE)
        && spitter.atSpeed(CargoConstants.SHOOTER_TOLERANCE);
  }

  /** Prepares the shooter without actually shooting, minimizes shooting time */
  public Command ready() {
    return new InstantCommand(this::runIntakeReverse)
        .andThen(new WaitCommand(CargoConstants.REVERSE_BEFORE_SHOOT_TIME))
        .andThen(this::stop)
        .andThen(this::spinUp);
  }
  /**
   * Create a command that spins up the spitter (and shooter, if the hood is on), waits till they're
   * at a speed to shoot, then spits/shoot
   */
  public Command startShooterCommand() {
    var preSpinUp =
        new ConditionalCommand(
            new WaitCommand(0),
            new InstantCommand(this::runIntakeReverse, this)
                .andThen(new WaitCommand(CargoConstants.REVERSE_BEFORE_SHOOT_TIME)),
            () -> this.isReady);
    var shootCommand =
        preSpinUp
            .andThen(() -> intakeMotor.set(0), this)
            .andThen(this::spinUp, this)
            .andThen(
                new WaitCommand(CargoConstants.SHOOT_HIGH_SPINUP_TIME).withInterrupt(this::atSpeed))
            .andThen(this::startFeeding, this);
    var spitCommand = new InstantCommand(this::startFeeding, this).andThen(this::spinUp, this);
    return new ConditionalCommand(shootCommand, spitCommand, this::hoodOn);
  }

  /** Stop all motors */
  public void stop() {
    intakeMotor.set(0);
    spitter.stop();
    shooter.stop();
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

  /** Set the hood solenoid to off */
  public void turnHoodOff() {
    deployHood.set(DoubleSolenoid.Value.kOff);
  }

  /** Is the hood on to shoot or is it out of the way to spit? */
  private boolean hoodOn() {
    return deployHood.get() != DoubleSolenoid.Value.kForward;
  }
}
