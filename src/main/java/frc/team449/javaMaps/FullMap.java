package frc.team449.javaMaps;

import edu.wpi.first.math.controller.ElevatorFeedforward;
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
import frc.team449.javaMaps.builders.SparkMaxConfig;
import frc.team449.other.DefaultCommand;
import frc.team449.other.Updater;
import frc.team449.wrappers.AHRS;
import frc.team449.wrappers.PDP;
import frc.team449.wrappers.RumbleableJoystick;
import org.jetbrains.annotations.NotNull;

import java.util.List;

public class FullMap {
  // Motor IDs
  public static final int RIGHT_LEADER_PORT = 2,
      RIGHT_LEADER_FOLLOWER_1_PORT = 3,
      LEFT_LEADER_PORT = 1,
      LEFT_LEADER_FOLLOWER_1_PORT = 4,
      INTAKE_LEADER_PORT = 5,
      INTAKE_FOLLOWER_PORT = 6,
      SPITTER_PORT = 7,
      CLIMBER_MOTOR_PORT = 5;

  // Controller ports
  public static final int MECHANISMS_JOYSTICK_PORT = 0, DRIVE_JOYSTICK_PORT = 1;
  // Button numbers
  public static final int INTAKE_NORMAL_BUTTON = 1, INTAKE_REVERSE_BUTTON = 3, SPIT_BUTTON = 2;
  // Speeds
  public static final double INTAKE_SPEED = 0.1, SPITTER_SPEED = 0.1;

  private FullMap() {}

  @NotNull
  public static RobotMap createRobotMap() {

    var pdp = new PDP(0, new RunningLinRegComponent(250, 0.75), PowerDistribution.ModuleType.kCTRE);

    var mechanismsJoystick = new Joystick(MECHANISMS_JOYSTICK_PORT);
    var driveJoystick = new RumbleableJoystick(DRIVE_JOYSTICK_PORT);
    List<GenericHID> joysticks = List.of(mechanismsJoystick, driveJoystick);

    var navx = new AHRS(SerialPort.Port.kMXP, true);

    var sparkPrototype =
        new SparkMaxConfig()
            .setEnableBrakeMode(true)
            .setUnitPerRotation(0.4787787204060999)
            .setCurrentLimit(50)
            .setEnableVoltageComp(true);

    class DummyDigitalInput extends DigitalInput {

      /**
       * Create an instance of a Digital Input class. Creates a digital input given a channel.
       *
       * @param channel the DIO channel for the digital input 0-9 are on-board, 10-25 are on the MXP
       */
      public DummyDigitalInput(int channel) {
        super(channel);
      }

      @Override
      public boolean get() {
        return false;
      }
    }

    var climber =
        new PivotingTelescopingClimber(
            sparkPrototype
                .copy()
                .setName("climber_motor")
                .setPort(CLIMBER_MOTOR_PORT)
                .setUnitPerRotation(1)
                .createReal(),
            /*new SolenoidSimple(new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1))*/ null,
            new DummyDigitalInput(0),
            new DummyDigitalInput(1),
            new ElevatorFeedforward(0, 0, 0, 0),
            1,
            0,
            0,
            5, // 1 rot/s max vel,
            .5, // .5 rot/s^2
            20 // rotations
            );

    // PUT YOUR SUBSYSTEM IN HERE AFTER INITIALIZING IT
    var subsystems = List.<Subsystem>of(climber);
    var updater = new Updater(List.of(pdp, navx));

    var defaultCommands = List.<DefaultCommand>of();

    // TODO BUTTON BINDINGS HERE
    new JoystickButton(mechanismsJoystick, XboxController.Button.kY.value)
        .whenPressed(new ExtendTelescopingArm(climber));
    new JoystickButton(mechanismsJoystick, XboxController.Button.kA.value)
        .whenPressed(new RetractTelescopingArm(climber));
    //    new JoystickButton(mechanismsJoystick, XboxController.Button.kX.value)
    //        .whenPressed(climber::pivotTelescopingArmIn, climber);
    //    new JoystickButton(mechanismsJoystick, XboxController.Button.kB.value)
    //        .whenPressed(climber::pivotTelescopingArmOut, climber);

    List<Command> robotStartupCommands = List.of();

    List<Command> autoStartupCommands = List.of();

    List<Command> teleopStartupCommands = List.of();

    List<Command> testStartupCommands = List.of();
    var allCommands =
        new CommandContainer(
            robotStartupCommands, autoStartupCommands, teleopStartupCommands, testStartupCommands);

    return new RobotMap(subsystems, pdp, updater, allCommands, joysticks, false);
  }
}
