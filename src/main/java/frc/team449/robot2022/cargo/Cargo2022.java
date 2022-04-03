package frc.team449.robot2022.cargo;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449.motor.WrappedMotor;
import org.jetbrains.annotations.NotNull;

public class Cargo2022 extends SubsystemBase {
  /** The leader motor for the intake */
  private final WrappedMotor intakeMotor;
  /** The top motor that lets balls be spit out */
  private final WrappedMotor spitterMotor;
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
  private double flywheelSpeed = 0;
  /** Tracks the desired speed of the spitter */
  private double spitterSpeed = 0;

  public Cargo2022(
      @NotNull WrappedMotor intakeMotor,
      @NotNull WrappedMotor spitterMotor,
      @NotNull SimpleMotorFeedforward spitterFF,
      @NotNull WrappedMotor flywheelMotor,
      @NotNull SimpleMotorFeedforward flywheelFF,
      @NotNull DoubleSolenoid deployIntake,
      @NotNull DoubleSolenoid deployHood) {
    this.intakeMotor = intakeMotor;
    this.spitterMotor = spitterMotor;
    this.spitterFF = spitterFF;
    this.flywheelMotor = flywheelMotor;
    this.flywheelFF = flywheelFF;
    this.deployIntake = deployIntake;
    this.deployHood = deployHood;
  }

  public void runIntake() {
    intakeMotor.set(CargoConstants.FEEDER_OUTPUT);
    spitterSpeed = -CargoConstants.SPITTER_INTAKE_SPEED_RPS;
  }

  public void runIntakeReverse() {
    intakeMotor.set(-CargoConstants.FEEDER_OUTPUT);
    spitterSpeed = -CargoConstants.SPITTER_INTAKE_SPEED_RPS;
  }

  /**
   * Run the intake belts (but don't change the spitter and flywheel speeds) so the balls move up
   * and are ready to spit/shoot
   */
  private void startFeeding() {
    intakeMotor.set(CargoConstants.FEEDER_OUTPUT);
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
      spitterSpeed = CargoConstants.SPITTER_SPEED_RPS;
      flywheelSpeed = 0;
    }
  }

  /**
   * Create a command that spins up the spitter (and flywheel, if the hood is on), waits till
   * they're at a speed to shoot, then spits/shoot
   */
  public Command startShooterCommand() {
    var shootCommand =
        new InstantCommand(this::runIntakeReverse)
            .andThen(new WaitCommand(CargoConstants.REVERSE_BEFORE_SHOOT_TIME))
            .andThen(this::stop)
            .andThen(this::spinUp)
            .andThen(new WaitCommand(CargoConstants.SHOOT_HIGH_SPINUP_TIME))
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
    double flywheelDelta = flywheelSpeed - flywheelMotor.getVelocity();
    double spitterDelta = spitterSpeed - spitterMotor.getVelocity();
    flywheelMotor.setVoltage(flywheelFF.calculate(flywheelSpeed, flywheelDelta / 0.02));
    spitterMotor.setVoltage(spitterFF.calculate(spitterSpeed, spitterDelta / 0.02));
  }
}
