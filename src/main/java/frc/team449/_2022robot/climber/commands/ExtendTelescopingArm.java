package frc.team449._2022robot.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449._2022robot.climber.PivotingTelescopingClimber;

public class ExtendTelescopingArm extends CommandBase {
    private final PivotingTelescopingClimber climber;

    public ExtendTelescopingArm(PivotingTelescopingClimber climber) {
        addRequirements(climber);
        this.climber = climber;
    }

    @Override
    public void initialize(){
        climber.setGoal(climber.getMeasurement() + climber.DISTANCE_TOP_BOTTOM);
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted)
            System.out.println("Successfully extended climber!");
        climber.setGoal(climber.getMeasurement());
    }

    @Override
    public boolean isFinished(){
        return climber.topLimitSwitchTriggered();
    }
}
