package frc.team449.robot2022.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.List;

public class ClimberAutomatedTest {
    double climberExtendTime = 2.1;
    double climberRetractTime = 1.5;
    double moveForwardWaitTime = 6.9;

    List<Command> climberAutomatedCommands;
    public List<Command> climberAutomated(Climber2022 climber) {
        climberAutomatedCommands = List.of(
            new RunCommand(climber::setExtend).withTimeout(climberExtendTime)
                    .andThen(climber::stop)
                    .andThen(new WaitCommand(moveForwardWaitTime))
                    .andThen(new RunCommand(climber::setRetract).withTimeout(climberRetractTime))
        );
        return climberAutomatedCommands;
    }
}
