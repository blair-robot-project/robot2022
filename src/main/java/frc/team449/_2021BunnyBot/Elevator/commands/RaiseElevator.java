package frc.team449._2021BunnyBot.Elevator.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449._2021BunnyBot.Elevator.OneMotorPulleyElevator;


public class RaiseElevator extends CommandBase {
    private final OneMotorPulleyElevator elevator;
    public RaiseElevator(OneMotorPulleyElevator elevator) {
        addRequirements(elevator);
        this.elevator = elevator;
    }
    @Override
    public void execute(){
        elevator.setSameVelocity();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
