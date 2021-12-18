package frc.team449.javaMaps;

import edu.wpi.first.wpilibj.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449._2021BunnyBot.elevator.OneMotorPulleyElevator;
import frc.team449._2021BunnyBot.elevator.OneMotorPulleyElevator.ElevatorPosition;
import frc.team449._2021BunnyBot.elevator.commands.MoveToPosition;
import frc.team449._2021BunnyBot.elevator.commands.SetVelocity;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.generalInterfaces.shiftable.Shiftable;
import frc.team449.jacksonWrappers.MappedJoystick;
import frc.team449.jacksonWrappers.MappedSparkMax;
import frc.team449.jacksonWrappers.PDP;
import frc.team449.javaMaps.builders.PerGearSettingsBuilder;
import frc.team449.javaMaps.builders.SmartMotorConfig;
import frc.team449.oi.buttons.CommandButton;
import frc.team449.oi.buttons.SimpleButton;
import frc.team449.other.DefaultCommand;
import frc.team449.other.Updater;
import org.jetbrains.annotations.NotNull;

import java.util.List;

public class PositionControlTest {
  // Drive system
  public static final int LEFT_MASTER_PORT = 1,
      LEFT_MASTER_SLAVE_1_PORT = 3,
      LEFT_MASTER_SLAVE_2_PORT = 5,
      RIGHT_MASTER_PORT = 2,
      RIGHT_MASTER_SLAVE_1_PORT = 4,
      RIGHT_MASTER_SLAVE_2_PORT = 6;
  // Solenoid ports
  public static final int INTAKE_SOLENOID_FORWARD_PORT = 2, INTAKE_SOLENOID_REVERSE_PORT = 3;
  public static final int MECHANISMS_JOYSTICK_PORT = 0, DRIVE_JOYSTICK_PORT = 1;

  private PositionControlTest() {
    throw new IllegalStateException("This is a utility class!");
  }

  @NotNull
  public static RobotMap createRobotMap() {
    // TODO Declare these constants outside this method and remove unused variables

    // Motor ports
    int elevatorMotorPort = 5;

    // Solenoid ports
    int intakeSolenoidForward = 2, intakeSolenoidReverse = 3;

    // Drive input-output ports. Things like encoders go here

    // Joystick ports
    int mechanismsJoystickPort = 0, driveJoystickPort = 1;

    // Driver button numbers
    int driverIntakeOutOn = 1,
        driverIntakeOff = 2, // TODO This is never used
        driverIntakeRev = 3, // TODO This is never used
        driverIntakeInOff = 4, // TODO This is never used
        shiftUp = 5;

    // Mechs button numbers
    int elevatorMoveToTop = 1,
        elevatorMoveToUpper = 2,
        elevatorMoveToLower = 3,
        elevatorMoveToBottom = 4,
        intakeClose = 7,
        intakeOpen = 8;
    // Motor speeds
    double elevatorMaxVelocity = .05; // TODO this is a placeholder

    var useCameraServer = false;
    var pdp = new PDP(0, new RunningLinRegComponent(250, 0.75));

    var mechanismsJoystick = new MappedJoystick(mechanismsJoystickPort);
    //        var driveJoystick = new MappedJoystick(driveJoystickPort);
    var joysticks = List.of(mechanismsJoystick /*, driveJoystick*/);
    // Elevator
    var elevatorPulleyMotor =
          new MappedSparkMax(
            null,
            null,
            new SmartMotorConfig()
                .setName("elevator")
                .setPort(elevatorMotorPort)
                .setReverseOutput(false)
                .setEnableBrakeMode(true)
                .setPdp(pdp)
                .setCurrentLimit(40)
                .setEnableVoltageComp(false)
                .setPerGearSettings(
                    List.of(
                        new PerGearSettingsBuilder()
                            .gear(Shiftable.Gear.LOW)
                            .maxSpeed(elevatorMaxVelocity)
                            .build()))
                .ensureBuilt());
    // PID constants for velocity controlled elevator motor
    //    elevatorPulleyMotor.setPID(0.0003, 0.0000008, 0.0146);
    // PID constants for position controlled elevator motor
    //    elevatorPulleyMotor.setPID(0.2, 0.0008, 0.016);
    elevatorPulleyMotor.setPID(0.0, 0.0, 0.0);
    // WE ASSUME THE ELEVATOR STARTS AT THE BOTTOM
    // PLEASE MAKE SURE ELEVATOR IS ACTUALLY AT THE BOTTOM

    var elevator =
        new OneMotorPulleyElevator(
            elevatorPulleyMotor,
            ElevatorPosition.BOTTOM,
            new ElevatorFeedforward(0.11311, 0.15109, 3.8541, 0.30047), // TODO do characterization
            new TrapezoidProfile.Constraints(
                elevatorMaxVelocity, 0.1)); // TODO [IMPORTANT] These values are placeholders
    var setVelocityCommand = new SetVelocity(elevator, mechanismsJoystick, elevatorMaxVelocity);

    // intake
    /*var intake =
                new IntakeActuated(
                        new SolenoidSimple(new DoubleSolenoid(intakeSolenoidForward, intakeSolenoidReverse)));
    */
    var subsystems = List.<Subsystem>of(elevator);

    var updater = new Updater(List.of(pdp /*, oi, navx*/));

    var defaultCommands = List.<DefaultCommand>of();

    var buttons =
        List.of(
            // elevator move to TOP position
            new CommandButton(
                new SimpleButton(mechanismsJoystick, elevatorMoveToTop),
                new MoveToPosition(ElevatorPosition.TOP, elevator),
                CommandButton.Action.WHEN_PRESSED),
            // elevator move to UPPER position
            new CommandButton(
                new SimpleButton(mechanismsJoystick, elevatorMoveToUpper),
                new MoveToPosition(ElevatorPosition.UPPER, elevator),
                CommandButton.Action.WHEN_PRESSED),
            // elevator move to LOWER position
            new CommandButton(
                new SimpleButton(mechanismsJoystick, elevatorMoveToLower),
                new MoveToPosition(ElevatorPosition.LOWER, elevator),
                CommandButton.Action.WHEN_PRESSED),
            new CommandButton(
                new SimpleButton(mechanismsJoystick, elevatorMoveToBottom),
                new MoveToPosition(ElevatorPosition.BOTTOM, elevator),
                CommandButton.Action.WHEN_PRESSED));

    var robotStartupCommands = List.<Command>of();
    var autoStartupCommands = List.<Command>of();
    var teleopStartupCommands = List.<Command>of();
    var testStartupCommands = List.<Command>of();
    var allCommands =
        new CommandContainer(
            defaultCommands,
            buttons,
            robotStartupCommands,
            autoStartupCommands,
            teleopStartupCommands,
            testStartupCommands);
    // Shuffleboard buttons
    SmartDashboard.putData("Move to Top", new MoveToPosition(ElevatorPosition.TOP, elevator));
    SmartDashboard.putData("Move to Upper", new MoveToPosition(ElevatorPosition.UPPER, elevator));
    SmartDashboard.putData("Move to Lower", new MoveToPosition(ElevatorPosition.LOWER, elevator));
    SmartDashboard.putData("Move to Bottom", new MoveToPosition(ElevatorPosition.BOTTOM, elevator));

    return new RobotMap(subsystems, pdp, updater, allCommands, joysticks, useCameraServer);
  }
}
