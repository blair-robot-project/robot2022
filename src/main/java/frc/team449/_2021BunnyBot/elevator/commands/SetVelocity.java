package frc.team449._2021BunnyBot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.Converter;
import frc.team449._2021BunnyBot.elevator.OneMotorPulleyElevator;
import frc.team449.jacksonWrappers.MappedJoystick;

public class SetVelocity extends CommandBase {
  private final OneMotorPulleyElevator elevator; // The elevator to control
  private final MappedJoystick joystick; // The joystick to read
  private final double maxVelocity; // The elevator's max allowed velocity

  private static final double minInput = 0.01;

  public SetVelocity(OneMotorPulleyElevator elevator, MappedJoystick joystick, double maxVelocity) {
    addRequirements(elevator);
    this.elevator = elevator;
    this.joystick = joystick;
    this.maxVelocity = maxVelocity;
  }

  /**
   * Set the velocity of the elevator to the result of passing the joystick's {@link
   * MappedJoystick#getY() getY} through {@link Converter#joystickInputToVelocity(double, double)
   * joystickInputToVelocity}, unless the value is <0.07
   */
  @Override
  public void execute() {
    double joystickValue = -joystick.getY();
    // Ignore anything <1% of the max value
    if (Math.abs(joystickValue) >= minInput) {
      // Set the elevator velocity to the joystick value run through the converter
      var converted = Converter.joystickInputToVelocity(joystickValue, maxVelocity);
      System.out.println("Joystick Y value: " + converted);
      elevator.setVelocityUPS(converted);
    } else { // set to 0 if input is <1%
      elevator.setVelocityUPS(0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
