package frc.team449._2021BunnyBot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449._2021BunnyBot.intake.OnePistonIntake;
import frc.team449._2021BunnyBot.intake.OnePistonIntake.*;
import org.jetbrains.annotations.NotNull;

/**
 * This command takes a position to set the intake to (open or closed) Then sets the position by
 * calling close() or open() in the OnePistonIntake.java
 */
public class SetIntake extends CommandBase {
  @NotNull private final IntakePosition newPosition;
  @NotNull private final OnePistonIntake intake;

  public SetIntake(@NotNull IntakePosition newPosition, @NotNull OnePistonIntake intake) {
    addRequirements(intake);
    this.newPosition = newPosition;
    this.intake = intake;
  }

  /** On init, sends the position that the intake is being set to */
  @Override
  public void initialize() {
    System.out.println(
        "Setting the position of the intake to : "
            + (newPosition == IntakePosition.OPEN ? "OPEN" : "CLOSED"));
  }

  /** Sets the position of the intake to open or closed */
  @Override
  public void execute() {
    if (newPosition == IntakePosition.OPEN) {
      intake.open();
    } else {
      intake.close();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
