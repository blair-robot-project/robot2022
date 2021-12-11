package frc.team449._2021BunnyBot.elevator.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449._2021BunnyBot.elevator.OneMotorPulleyElevator;
import org.jetbrains.annotations.NotNull;

public class MoveToPosition extends CommandBase {
  private final OneMotorPulleyElevator.ElevatorPosition position;
  private final OneMotorPulleyElevator elevator;
  private final ProfiledPIDController controller;

  public MoveToPosition(
      @NotNull OneMotorPulleyElevator.ElevatorPosition position,
      @NotNull OneMotorPulleyElevator elevator) {
    this.addRequirements(elevator);
    this.elevator = elevator;
    this.position = position;
    controller = elevator.getController();
  }

  /** Moves to designated position for command */
  @Override
  public void execute() {
    System.out.println("Moving to " + position + " position.");
    // If there's already a MoveToPosition command running on the elevator, cancel it
    //    var currCmd = CommandScheduler.getInstance().requiring(this.elevator);
    //    if (currCmd instanceof MoveToPosition && ((MoveToPosition) currCmd).position !=
    // this.position) {
    //      currCmd.cancel();
    //    }
    //    elevator.moveToPosition(this.position);
    elevator.moveToPosition(position);
  }
  /** Some tolerance, stops if elevator is within .01 meters of the setpoint */
  @Override
  public boolean isFinished() {
    return Math.abs(elevator.getRawPosition() - position.distanceFromBottom) < .01;
  }
}
