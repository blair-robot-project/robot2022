package frc.team449._2022robot.cargo;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jetbrains.annotations.NotNull;

public class Cargo2022 extends SubsystemBase {
  /** The leader motor for the intake */
  public final MotorController intakeMotor;
  /** The top motor that lets balls be spit out */
  public final MotorController spitterMotor;
  /** Piston used to extend and retract intake */
  private final DoubleSolenoid deployIntake;
  /** The speed when intaking */
  private final double intakeSpeed;
  /** The speed of the spitter motor when spitting */
  private final double spitterSpeed;

  public Cargo2022(
      @NotNull MotorController intakeMotor,
      @NotNull MotorController spitterMotor,
      @NotNull DoubleSolenoid deployIntake,
      double intakeSpeed,
      double spitterSpeed) {
    this.intakeMotor = intakeMotor;
    this.spitterMotor = spitterMotor;
    this.deployIntake = deployIntake;
    this.intakeSpeed = intakeSpeed;
    this.spitterSpeed = spitterSpeed;
  }

  public void runIntake() {
    intakeMotor.set(intakeSpeed);
    spitterMotor.set(-spitterSpeed);
  }

  public void runIntakeReverse() {
    intakeMotor.set(-intakeSpeed);
    spitterMotor.set(-spitterSpeed);
  }

  public void spit() {
    intakeMotor.set(intakeSpeed);
    spitterMotor.set(spitterSpeed);
  }

  public void stop() {
    intakeMotor.set(0);
    spitterMotor.set(0);
  }

  public void deployIntake() {
    deployIntake.set(DoubleSolenoid.Value.kReverse);
  }

  public void retractIntake() {
    deployIntake.set(DoubleSolenoid.Value.kForward);
  }
}
