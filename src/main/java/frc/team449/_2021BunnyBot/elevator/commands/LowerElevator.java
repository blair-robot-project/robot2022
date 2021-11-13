package frc.team449._2021BunnyBot.elevator.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449._2021BunnyBot.elevator.OneMotorPulleyElevator;
import frc.team449._2021BunnyBot.elevator.OneMotorPulleyElevator.ElevatorPosition;

public class LowerElevator extends CommandBase {
    private final OneMotorPulleyElevator elevator;
    public LowerElevator(OneMotorPulleyElevator elevator) {
        addRequirements(elevator);
        this.elevator = elevator;
    }
    /**
     * Moves the elevator one below the current level of elevation
     */
    @Override

    public void execute(){
        switch(elevator.getPosition()){
            case TOP: elevator.moveToPosition(ElevatorPosition.UPPER);break;
            case UPPER: elevator.moveToPosition(ElevatorPosition.LOWER);break;
            case LOWER: elevator.moveToPosition(ElevatorPosition.BOTTOM);break;
            // Case bottom does not exist because we cannot move lower than the bottom of the elevator
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
