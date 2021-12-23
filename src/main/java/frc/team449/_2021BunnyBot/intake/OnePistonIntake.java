package frc.team449._2021BunnyBot.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.multiSubsystem.SolenoidSimple;
import org.jetbrains.annotations.NotNull;

public class OnePistonIntake extends SubsystemBase {
  // one piston needed for the intake
  private final SolenoidSimple intakePiston;

  public OnePistonIntake(@NotNull SolenoidSimple intakePiston) {
    this.intakePiston = intakePiston;
  }
  /** closes the two arms together */
  public void close() {
    // Switch to reverse channel
    intakePiston.setSolenoid(DoubleSolenoid.Value.kReverse);
  }
  /** opens the two arms apart */
  public void open() {
    // Switch to forward channel
    intakePiston.setSolenoid(DoubleSolenoid.Value.kForward);
  }

  /**
   * We don't want the intake to be off, just: OPEN -> forward channel (piston out) or CLOSED ->
   * reverse channel (piston in)
   */
  public enum IntakePosition {
    OPEN(DoubleSolenoid.Value.kForward),
    CLOSED(DoubleSolenoid.Value.kReverse);

    final DoubleSolenoid.Value intakePosition;

    IntakePosition(DoubleSolenoid.Value intakePosition) {
      this.intakePosition = intakePosition;
    }
  }
}
