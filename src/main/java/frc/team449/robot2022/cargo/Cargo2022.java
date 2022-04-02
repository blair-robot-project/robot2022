package frc.team449.robot2022.cargo;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jetbrains.annotations.NotNull;

public class Cargo2022 extends SubsystemBase {
  /** The leader motor for the intake */
  private final MotorController intakeMotor;
  /** The top motor that lets balls be spit out */
  private final MotorController spitterMotor;
  /** Motor used for shooting flywheel */
  private final MotorController flywheelMotor;
  /** Piston used to extend and retract intake */
  private final DoubleSolenoid deployIntake;
  /** Piston used to deploy and remove hood */
  private final DoubleSolenoid deployHood;
  /** Whether or not shooter is on */
  private boolean isShooting = false;

  public Cargo2022(
      @NotNull MotorController intakeMotor,
      @NotNull MotorController spitterMotor,
      @NotNull MotorController flywheelMotor,
      @NotNull DoubleSolenoid deployIntake,
      @NotNull DoubleSolenoid deployHood) {
    this.intakeMotor = intakeMotor;
    this.spitterMotor = spitterMotor;
    this.flywheelMotor = flywheelMotor;
    this.deployIntake = deployIntake;
    this.deployHood = deployHood;
  }

  public void runIntake() {
    intakeMotor.set(CargoConstants.FEEDER_OUTPUT);
    spitterMotor.set(-CargoConstants.SPITTER_OUTPUT);
    flywheelMotor.set(0);
  }

  public void runIntakeReverse() {
    intakeMotor.set(-CargoConstants.FEEDER_OUTPUT);
    spitterMotor.set(-CargoConstants.SPITTER_OUTPUT);
    flywheelMotor.set(0);
  }

  /** If hood is on, shoots high. Otherwise, spits low */
  public void spitOrShoot() {
    if (hoodOn()) {
      this.shoot();
    } else {
      this.spit();
    }
  }

  /** If hood is on, shoots high. Otherwise, spits low */
  public void spit() {
    intakeMotor.set(CargoConstants.FEEDER_OUTPUT);
    spitterMotor.set(CargoConstants.SPITTER_OUTPUT);
    flywheelMotor.set(0);
  }

  /** If hood is on, shoots high. Otherwise, spits low */
  public void shoot() {
    intakeMotor.set(CargoConstants.FEEDER_OUTPUT);
    spitterMotor.set(CargoConstants.SPITTER_SHOOT_OUTPUT);
    flywheelMotor.set(CargoConstants.SHOOTER_OUTPUT);
  }

  /**
   * If not already shooting and hood is on, run all motors at the outputs required for shooting.
   * Otherwise, stop all motors.
   */
  public void toggleShoot() {
    if (isShooting || !this.hoodOn()) {
      this.stop();
    } else {
      this.shoot();
    }
  }

  public void stop() {
    intakeMotor.set(0);
    spitterMotor.set(0);
    flywheelMotor.set(0);
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

  private boolean hoodOn() {
    return deployHood.get() == DoubleSolenoid.Value.kReverse;
  }
}
