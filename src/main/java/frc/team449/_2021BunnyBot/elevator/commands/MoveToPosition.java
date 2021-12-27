package frc.team449._2021BunnyBot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449._2021BunnyBot.elevator.OneMotorPulleyElevator;
import org.jetbrains.annotations.NotNull;

public class MoveToPosition extends CommandBase {
  private final OneMotorPulleyElevator.ElevatorPosition position;
  private final OneMotorPulleyElevator elevator;
  private double startTime;

  public MoveToPosition(
      @NotNull OneMotorPulleyElevator.ElevatorPosition position,
      @NotNull OneMotorPulleyElevator elevator) {
    this.addRequirements(elevator);
    this.elevator = elevator;
    this.position = position;
  }
  // runs when the command starts
  @Override
  public void initialize() {
    System.out.println("[INITIALIZING] Moving to " + position + " position.");
    startTime = System.currentTimeMillis();
  }
  /** Moves to designated position for command */
  @Override
  public void execute() {
    elevator.moveToPosition(position, System.currentTimeMillis() - startTime);
  }

  /** Prints to the console if it was interrupted or successfully moved to position */
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      System.out.println("[SUCCESSFUL] Moved to " + position + " position.");
    } else {
      System.out.println("[UNSUCCESSFUL] Attempted to move to " + position + " position.");
    }
  }
  /** Some tolerance, stops if elevator is within .0075 meters of the setpoint */
  @Override
  public boolean isFinished() {
    return Math.abs(elevator.getPositionUnits() - position.distanceFromBottom) < 0.0075;
  }
}
