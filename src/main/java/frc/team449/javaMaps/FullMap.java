package frc.team449.javaMaps;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449._2022robot.climber.PivotingTelescopingClimber;
import frc.team449._2022robot.climber.commands.ExtendTelescopingArm;
import frc.team449._2022robot.climber.commands.RetractTelescopingArm;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.DriveAtSpeed;
import frc.team449.drive.unidirectional.commands.UnidirectionalNavXDefaultDrive;
import frc.team449.generalInterfaces.doubleUnaryOperator.Polynomial;
import frc.team449.generalInterfaces.doubleUnaryOperator.RampComponent;
import frc.team449.jacksonWrappers.MappedAHRS;
import frc.team449.jacksonWrappers.MappedJoystick;
import frc.team449.jacksonWrappers.PDP;
import frc.team449.jacksonWrappers.SlaveSparkMax;
import frc.team449.javaMaps.builders.DriveSettingsBuilder;
import frc.team449.javaMaps.builders.SparkMaxConfig;
import frc.team449.javaMaps.builders.ThrottlePolynomialBuilder;
import frc.team449.multiSubsystem.SolenoidSimple;
import frc.team449.oi.throttles.ThrottleSum;
import frc.team449.oi.unidirectional.arcade.OIArcadeWithDPad;
import frc.team449.other.Debouncer;
import frc.team449.other.DefaultCommand;
import frc.team449.other.Updater;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Map;
import java.util.Set;

public class FullMap {
  // Motor IDs
  public static final int RIGHT_LEADER_PORT = 2,
      RIGHT_LEADER_FOLLOWER_1_PORT = 3,
      LEFT_LEADER_PORT = 1,
      LEFT_LEADER_FOLLOWER_1_PORT = 4;
  // Controller ports
  public static final int MECHANISMS_JOYSTICK_PORT = 0, DRIVE_JOYSTICK_PORT = 1;

  // TODO PUT ADDITIONAL CONSTANTS HERE

  private FullMap() {}

  @NotNull
  public static RobotMap createRobotMap() {

    var pdp = new PDP(1, new RunningLinRegComponent(250, 0.75), PowerDistribution.ModuleType.kCTRE);

    var mechanismsJoystick = new MappedJoystick(MECHANISMS_JOYSTICK_PORT);
    var driveJoystick = new MappedJoystick(DRIVE_JOYSTICK_PORT);
    List<GenericHID> joysticks = List.of(mechanismsJoystick, driveJoystick);

    var navx = new MappedAHRS(SerialPort.Port.kMXP, true);

    var SparkPrototype =
        new SparkMaxConfig()
            .setEnableBrakeMode(true)
            .setUnitPerRotation(1)
            .setCurrentLimit(50)
            .setEnableVoltageComp(true);

    var climber = new PivotingTelescopingClimber(
            SparkPrototype.copy().setName("climber_motor").createReal(),
            new SolenoidSimple(new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1)),
            new DigitalInput(0),
            new DigitalInput(1),
            new ElevatorFeedforward(0,0,0,0),
            1,
            0,
            0,
            1, // 1 m/s max vel
            .01, // 1 cm/s^2
            1.2 //meters
    );
    var subsystems =
        List.<Subsystem>of(climber); // TODO PUT YOUR SUBSYSTEM IN HERE AFTER INITIALIZING IT

    var updater = new Updater(List.of(pdp, navx));

    var defaultCommands = List.<DefaultCommand>of();

    // TODO BUTTON BINDINGS HERE
    new JoystickButton(mechanismsJoystick, 1).whenPressed(new ExtendTelescopingArm(climber));
    new JoystickButton(mechanismsJoystick, 2).whenPressed(new RetractTelescopingArm(climber));

    List<Command> robotStartupCommands = List.of();

    List<Command> autoStartupCommands =
        List.of();

    List<Command> teleopStartupCommands = List.of();

    List<Command> testStartupCommands = List.of();
    var allCommands =
        new CommandContainer(
            defaultCommands,
            robotStartupCommands,
            autoStartupCommands,
            teleopStartupCommands,
            testStartupCommands);

    return new RobotMap(subsystems, pdp, updater, allCommands, joysticks, false);
  }
}
