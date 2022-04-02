package frc.team449.robot2022.cargo;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.motor.WrappedMotor;
import org.jetbrains.annotations.NotNull;

import javax.lang.model.util.SimpleElementVisitor6;

public class Cargo2022 extends SubsystemBase {
  /** The leader motor for the intake */
  private final WrappedMotor intakeMotor;
  /** The top motor that lets balls be spit out */
  private final WrappedMotor spitterMotor;
  /** Feed forward for the spitter*/
  private final SimpleMotorFeedforward spitterFF;
  /** Motor used for shooting flywheel */
  private final WrappedMotor flywheelMotor;
  /** Tracks the desired speed of the flywheel*/
  private double flywheelSpeed = 0;
  /** Tracks the desired speed of the spitter */
  private double spitterSpeed = 0;
  /** Feed forward for the flywheel*/
  private final SimpleMotorFeedforward flywheelFF;
  /** Piston used to extend and retract intake */
  private final DoubleSolenoid deployIntake;
  /** Piston used to deploy and remove hood */
  private final DoubleSolenoid deployHood;
  /** Bang bang controller for both the shooter and spitter*/
  private final BangBangController bangBangController = new BangBangController();

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

  /** If hood is on, shoots high. Otherwise, spits low */
//  public void spitOrShoot() {
//    if (hoodOn()) {
//      this.shoot();
//    } else {
//      this.spit();
//    }
//  }
//
//  /** If hood is on, shoots high. Otherwise, spits low */
//  public void spit() {
//    intakeMotor.set(CargoConstants.FEEDER_OUTPUT);
//    spitterSpeed = CargoConstants.SPITTER_SPEED_RPS;
//    flywheelSpeed = 0;
//  }
  public boolean atSpeed(){
    return Math.abs(flywheelSpeed - flywheelMotor.encoder.getVelocityUnits()) < 4.5
           && Math.abs(spitterSpeed - spitterMotor.encoder.getVelocityUnits()) < 4.5;
  }
  /** If hood is on, shoots high. Otherwise, spits low */
  public void shoot() {
    intakeMotor.set(CargoConstants.FEEDER_OUTPUT);
  }

  public void spinUp(){
    if (hoodOn()) {
      spitterSpeed = CargoConstants.SPITTER_SHOOT_SPEED_RPS;
      flywheelSpeed = CargoConstants.SHOOTER_SPEED_RPS;
    }else{
      spitterSpeed = CargoConstants.SPITTER_SPEED_RPS;
      flywheelSpeed = 0;
    }
  }
  /**
   * If not already shooting and hood is on, run all motors at the outputs required for shooting.
   * Otherwise, stop all motors.
   */
//  public void shootWithFlywheel() {
//    if (!this.hoodOn()) {
//      flywheelSpeed = 0;
//    } else {
//      shoot();
//    }
//  }

  public void stop() {
    intakeMotor.set(0);
    spitterSpeed = 0;
    flywheelSpeed = 0;
  }

  public void stopFlywheel(){
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

  private boolean hoodOn() {
    return deployHood.get() == DoubleSolenoid.Value.kReverse;
  }

  @Override
  public void periodic() {
    flywheelMotor.setVoltage(
//        bangBangController.calculate(flywheelMotor.encoder.getPositionUnits(), 0 * flywheelSpeed) * 12.0 +
        flywheelFF.calculate(flywheelSpeed)
    );
    spitterMotor.setVoltage(
//        bangBangController.calculate(spitterMotor.encoder.getVelocityUnits(), 0 * spitterSpeed) * 12.0 +
        spitterFF.calculate(spitterSpeed)
    );
  }
}
