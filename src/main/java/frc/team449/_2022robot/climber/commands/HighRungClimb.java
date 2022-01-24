package frc.team449._2022robot.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449._2022robot.climber.ClimberActuated;
import org.jetbrains.annotations.NotNull;

public class HighRungClimb extends CommandBase {
  private final ClimberActuated climber;

  public HighRungClimb(@NotNull final ClimberActuated climber) {
    this.addRequirements(climber);
    this.climber = climber;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climber.pivotTelescopingArmOut();
    // try to reach the high rung
    climber.extendTelescopingArm();
    // hook onto the high rung
    climber.pivotTelescopingArmIn();
    // climb it
    climber.retractTelescopingArm();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
