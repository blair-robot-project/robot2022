package frc.team449._2021BunnyBot.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449._2020.multiSubsystem.SolenoidSimple;
import org.jetbrains.annotations.NotNull;

public class IntakeActuated extends SubsystemBase {
  // one piston for the intake
  private final SolenoidSimple intakePiston;

  public IntakeActuated(@NotNull SolenoidSimple intakePiston) {
    this.intakePiston = intakePiston;
  }
  /*
  closes the two arms together
  */
  public void close() {
    // Switch to reverse channel
    intakePiston.setSolenoid(DoubleSolenoid.Value.kReverse);
  }
  /*
  opens the two arms apart
  */
  public void open() {
    // Switch to forward channel
    intakePiston.setSolenoid(DoubleSolenoid.Value.kForward);
  }

  public enum IntakePosition {
    OPEN(DoubleSolenoid.Value.kForward),
    CLOSED(DoubleSolenoid.Value.kReverse);

    DoubleSolenoid.Value intakePosition;

    IntakePosition(DoubleSolenoid.Value intakePosition) {
      this.intakePosition = intakePosition;
    }
  }
}
