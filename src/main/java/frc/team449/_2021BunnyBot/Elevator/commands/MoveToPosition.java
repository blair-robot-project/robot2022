package frc.team449._2021BunnyBot.Elevator.commands;

import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449._2021BunnyBot.Elevator.OneMotorPulleyElevator;
import org.jetbrains.annotations.NotNull;

public class MoveToPosition extends CommandBase {
    private final OneMotorPulleyElevator.ElevatorPosition position;
    private final OneMotorPulleyElevator elevator;

    public MoveToPosition(@NotNull OneMotorPulleyElevator.ElevatorPosition position,
                          @NotNull OneMotorPulleyElevator elevator){
        this.elevator = elevator;
        this.position = position;
    }

    /**
     * Moves to designated position for command
     */
    @Override
    public void execute(){
        elevator.moveToPosition(position);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
