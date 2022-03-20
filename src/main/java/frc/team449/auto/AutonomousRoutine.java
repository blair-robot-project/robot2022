package frc.team449.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team449.auto.commands.AutonomousCommand;

import java.util.List;

public class AutonomousRoutine extends SequentialCommandGroup {

  double executionTime = 0;

  public AutonomousRoutine(List<AutonomousCommand> commandList) {
    for (AutonomousCommand command : commandList) {
      addCommands(command.getAutoCommand());
      executionTime += command.getRunTimeSeconds() == null ? 0 : command.getRunTimeSeconds();
    }
    if (executionTime >= 15) {
      DriverStation.reportWarning(
          "The selected autonomous routine exceeds an execution time of 15 seconds"
              + " Optimize the routine or it won't finish during play!",
          false);
    }
  }
}
