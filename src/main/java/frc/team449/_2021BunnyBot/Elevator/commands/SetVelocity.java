package frc.team449._2021BunnyBot.Elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449._2021BunnyBot.Elevator.OneMotorPulleyElevator;

public class SetVelocity extends CommandBase {
    private final OneMotorPulleyElevator elevator;
    private final double velocity;
    public SetVelocity(double velocity, OneMotorPulleyElevator elevator) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.velocity = velocity;
    }

    @Override
    public void execute(){
        elevator.setVelocity(velocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
