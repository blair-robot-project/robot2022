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
//  private final MotorController flywheelMotor;
  /** Piston used to extend and retract intake */
  private final DoubleSolenoid deployIntake;
  /** The speed when intaking */
  private final double intakeSpeed;
  /** The speed of the spitter motor when spitting */
  private final double spitterSpeed;

  public Cargo2022(
      @NotNull MotorController intakeMotor,
      @NotNull MotorController spitterMotor,
//      @NotNull MotorController flywheelMotor,
      @NotNull DoubleSolenoid deployIntake,
      double intakeSpeed,
      double spitterSpeed) {
    this.intakeMotor = intakeMotor;
    this.spitterMotor = spitterMotor;
//    this.flywheelMotor = flywheelMotor;
    this.deployIntake = deployIntake;
    this.intakeSpeed = intakeSpeed;
    this.spitterSpeed = spitterSpeed;
  }

  public void runIntake() {
    intakeMotor.set(intakeSpeed);
    spitterMotor.set(-spitterSpeed);
//    flywheelMotor.set(0);
  }

  public void runIntakeReverse() {
    intakeMotor.set(-intakeSpeed);
    spitterMotor.set(-spitterSpeed);
//    flywheelMotor.set(0);
  }

  public void spit() {
    intakeMotor.set(intakeSpeed);
    spitterMotor.set(spitterSpeed);
//    flywheelMotor.set(0);
  }

  public void shootHigh() {
    intakeMotor.set(intakeSpeed);
    spitterMotor.set(spitterSpeed);
//    flywheelMotor.set(CargoConstants.SHOOT_HIGH_OUTPUT);
  }

  public void stop() {
    intakeMotor.set(0);
    spitterMotor.set(0);
//    flywheelMotor.set(0);
  }

  public void deployIntake() {
    this.deployIntake.set(DoubleSolenoid.Value.kReverse);
  }

  public void retractIntake() {
    this.deployIntake.set(DoubleSolenoid.Value.kForward);
  }
}
