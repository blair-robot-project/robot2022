package frc.team449._2021BunnyBot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449._2021BunnyBot.intake.IntakeActuated;
import frc.team449._2021BunnyBot.intake.IntakeActuated.IntakePosition;
import org.jetbrains.annotations.NotNull;

public class SetIntake extends CommandBase {
  @NotNull private final IntakePosition newPosition;
  @NotNull private final IntakeActuated intake;

  public SetIntake(@NotNull IntakePosition newPosition, @NotNull IntakeActuated intake) {
    addRequirements(intake);
    this.newPosition = newPosition;
    this.intake = intake;
  }

  @Override
  public void initialize() {
    System.out.println(
        "Setting the position of the intake to "
            + (newPosition == IntakePosition.OPEN ? "OPEN" : "CLOSED"));
  }

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
