package frc.team449._2021BunnyBot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import frc.team449.Converter;
import frc.team449._2021BunnyBot.elevator.OneMotorPulleyElevator;
import frc.team449.generalInterfaces.doubleUnaryOperator.Polynomial;
import frc.team449.jacksonWrappers.MappedJoystick;
import frc.team449.javaMaps.builders.ThrottlePolynomialBuilder;
import frc.team449.oi.throttles.Throttle;
import frc.team449.oi.throttles.ThrottleSum;

import java.util.Map;

public class SetVelocity extends CommandBase {
  private final OneMotorPulleyElevator elevator; // The elevator to control
  private final MappedJoystick joystick; // The joystick to read
  private final double maxVelocity; // The elevator's max allowed velocity
    private final Throttle fwdThrottle;

  public SetVelocity(OneMotorPulleyElevator elevator, MappedJoystick joystick, double maxVelocity) {
    addRequirements(elevator);
    this.elevator = elevator;
    this.joystick = joystick;
    this.maxVelocity = maxVelocity;
  var throttlePrototype =
                new ThrottlePolynomialBuilder().stick(joystick).smoothingTimeSecs(0.04).scale(0.7);
        var rotThrottle =
                throttlePrototype
                        .axis(0)
                        .deadband(0.08)
                        .inverted(false)
                        .polynomial(new Polynomial(Map.of(1., 0.5), null))
                        .build();
        this.fwdThrottle =
                new ThrottleSum(
                        new Throttle[]{
                                throttlePrototype
                                        .axis(3)
                                        .deadband(0.05)
                                        .inverted(false)
                                        .polynomial(
                                                new Polynomial(
                                                        Map.of(
                                                                1., 2.,
                                                                2., 1.),
                                                        null))
                                        .build(),
                                throttlePrototype.axis(2).inverted(true).build()
                        });
    }

  /**
   * Set the velocity of the elevator to the result of passing the joystick's {@link
   * MappedJoystick#getY() getY} through {@link Converter#joystickInputToVelocity(double, double)
   * joystickInputToVelocity}, unless the value is <0.01
   */
  @Override
  public void execute() {
    double joystickValue = fwdThrottle.getValue();
    if (joystickValue
        >= 0.01) { // Ignore anything <1% of the max value (assuming joystick is mapped to -1 to 1)
      // Set the elevator velocity to the joystick value run through the converter
      elevator.setVelocity(Converter.joystickInputToVelocity(joystickValue, maxVelocity));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
