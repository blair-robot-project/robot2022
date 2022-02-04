package frc.team449._2022robot.cargo;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cargo2022 extends SubsystemBase {
  /** The leader motor for the intake */
  public final MotorController intakeMotor;
  /** The top motor that lets balls be spit out */
  public final MotorController spitterMotor;
  /** The speed when intaking */
  private final double intakeSpeed;
  /** The speed of the spitter motor when spitting */
  private final double spitterSpeed;

  public Cargo2022(
      MotorController intakeMotor,
      MotorController spitterMotor,
      double intakeSpeed,
      double spitterSpeed) {
    this.intakeMotor = intakeMotor;
    this.spitterMotor = spitterMotor;
    this.intakeSpeed = intakeSpeed;
    this.spitterSpeed = spitterSpeed;
  }

  public void runIntake() {
    intakeMotor.set(intakeSpeed);
    spitterMotor.set(spitterSpeed);
  }

  public void runIntakeReverse() {
    intakeMotor.set(-intakeSpeed);
    spitterMotor.set(spitterSpeed);
  }

  public void spit() {
    intakeMotor.set(intakeSpeed);
    spitterMotor.set(-spitterSpeed);
  }

  public void stop() {
    intakeMotor.set(0);
    spitterMotor.set(0);
  }
}
