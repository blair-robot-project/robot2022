package frc.team449._2021BunnyBot.Elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449._2021BunnyBot.Elevator.OneMotorPulleyElevator;
import frc.team449.jacksonWrappers.MappedJoystick;

public class SetVelocity extends CommandBase {
    private final OneMotorPulleyElevator elevator;
    private final MappedJoystick joystick;
    public SetVelocity(OneMotorPulleyElevator elevator, MappedJoystick joystick) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.joystick = joystick;
    }

    @Override
    public void execute(){
        elevator.setVelocity(joystick.getY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
