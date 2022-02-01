package frc.team449._2022robot.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449._2022robot.climber.PivotingTelescopingClimber;

public class RetractTelescopingArm extends CommandBase {
  private final PivotingTelescopingClimber climber;

  public RetractTelescopingArm(PivotingTelescopingClimber climber) {
    addRequirements(climber);
    this.climber = climber;
  }

  @Override
  public void initialize() {
    if (climber.getState() != PivotingTelescopingClimber.ClimberState.RETRACTED) {
      climber.setGoal(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      System.out.println("Successfully retracted climber!");
      climber.setState(PivotingTelescopingClimber.ClimberState.RETRACTED);
    } else {
      climber.setState(PivotingTelescopingClimber.ClimberState.MIDDLE);
    }
    climber.getController().reset(climber.getMeasurement());
  }

  @Override
  public boolean isFinished() {
    return climber.bottomLimitSwitchTriggered() || climber.getController().atGoal();
  }
}
