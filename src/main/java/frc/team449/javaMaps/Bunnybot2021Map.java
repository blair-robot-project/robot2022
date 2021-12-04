package frc.team449.javaMaps;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449._2020.multiSubsystem.SolenoidSimple;
import frc.team449._2020.multiSubsystem.commands.SetSolenoidPose;
import frc.team449._2021BunnyBot.elevator.OneMotorPulleyElevator;
import frc.team449._2021BunnyBot.elevator.OneMotorPulleyElevator.ElevatorPosition;
import frc.team449._2021BunnyBot.elevator.commands.MoveToPosition;
import frc.team449._2021BunnyBot.elevator.commands.SetVelocity;
import frc.team449._2021BunnyBot.intake.commands.SetIntake;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.components.ShiftComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyroShiftable;
import frc.team449.generalInterfaces.SmartMotor;
import frc.team449.generalInterfaces.doubleUnaryOperator.Polynomial;
import frc.team449.generalInterfaces.shiftable.Shiftable;
import frc.team449.generalInterfaces.shiftable.commands.ShiftGears;
import frc.team449.jacksonWrappers.FeedForwardCalculators.MappedFeedForwardCalculator;
import frc.team449.jacksonWrappers.*;
import frc.team449.javaMaps.builders.PerGearSettingsBuilder;
import frc.team449.javaMaps.builders.SmartMotorConfigObject;
import frc.team449.javaMaps.builders.ThrottlePolynomialBuilder;
import frc.team449.oi.buttons.CommandButton;
import frc.team449.oi.buttons.SimpleButton;
import frc.team449.oi.throttles.Throttle;
import frc.team449.oi.throttles.ThrottleSum;
import frc.team449.oi.unidirectional.arcade.OIArcadeWithDPad;
import frc.team449.other.DefaultCommand;
import frc.team449.other.Updater;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Map;

public class Bunnybot2021Map {
  private Bunnybot2021Map() {
    throw new IllegalStateException("This is a utility class!");
  }

  @NotNull
  public static RobotMap createRobotMap() {
    // TODO Declare these constants outside this method and remove unused variables

    // Motor ports
    int leftMasterPort = 1,
        leftMasterSlave1Port = 3,
        leftMasterSlave2Port = 5,
        rightMasterPort = 2,
        rightMasterSlave1Port = 4,
        rightMasterSlave2Port = 6,
        elevatorMotorPort = 8;

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
    double elevatorMaxVelocity = 1; // TODO this is a placeholder

    var useCameraServer = false;

    var pdp = new PDP(0, new RunningLinRegComponent(250, 0.75));

    var mechanismsJoystick = new MappedJoystick(mechanismsJoystickPort);
    var driveJoystick = new MappedJoystick(driveJoystickPort);
    var joysticks = List.of(mechanismsJoystick, driveJoystick);

    var navx = new MappedAHRS(SerialPort.Port.kMXP, true);
    var driveMasterPrototype =
        new SmartMotorConfigObject()
            .setType(SmartMotor.Type.SPARK)
            .setEnableBrakeMode(true)
            .setPdp(pdp)
            .setUnitPerRotation(0.47877872)
            .setCurrentLimit(50)
            .setEnableVoltageComp(true)
            .setStartingGear(Shiftable.Gear.LOW);
    //            .setEncoderCPR(256);
    var lowGear =
        new PerGearSettingsBuilder()
            .gear(Shiftable.Gear.LOW)
            .postEncoderGearing(0.0488998)
            .maxSpeed(2.3)
            .kP(0);
    var highGear =
        new PerGearSettingsBuilder()
            .gear(Shiftable.Gear.HIGH)
            .postEncoderGearing(0.12936611)
            .maxSpeed(5.2)
            .kP(0.000001);

    var leftMaster =
        MappedSparkMax.create(
            null,
            null,
            driveMasterPrototype
                .copy()
                .setPort(leftMasterPort)
                .setName("left")
                .setReverseOutput(true)
                .setSlaveSparks(
                    List.of(
                        new SlaveSparkMax(leftMasterSlave1Port, false, pdp),
                        new SlaveSparkMax(leftMasterSlave2Port, false, pdp)))
                .setPerGearSettings(
                    List.of(
                        lowGear
                            .feedForwardCalculator(
                                new MappedFeedForwardCalculator(0.128, 5.23, 0.0698))
                            .build(),
                        highGear
                            .feedForwardCalculator(
                                new MappedFeedForwardCalculator(0.156, 2.01, 0.154))
                            .build()))
                .build());
    var rightMaster =
        MappedSparkMax.create(
            null,
            null,
            driveMasterPrototype
                .copy()
                .setName("right")
                .setPort(rightMasterPort)
                .setReverseOutput(false)
                .setSlaveSparks(
                    List.of(
                        new SlaveSparkMax(rightMasterSlave1Port, false, pdp),
                        new SlaveSparkMax(rightMasterSlave2Port, false, pdp)))
                .setPerGearSettings(
                    List.of(
                        lowGear
                            .feedForwardCalculator(
                                new MappedFeedForwardCalculator(0.139, 5.17, 0.0554))
                            .build(),
                        highGear
                            .feedForwardCalculator(
                                new MappedFeedForwardCalculator(0.165, 2.01, 0.155))
                            .build()))
                .build());
    var drive =
        new DriveUnidirectionalWithGyroShiftable(
            leftMaster,
            rightMaster,
            navx,
            0.61755,
            new ShiftComponent(
                List.of(leftMaster, rightMaster), new DoubleSolenoid(0, 0, 1), Shiftable.Gear.LOW),
            false);

    // Elevator
    var elevatorPulleyMotor =
        MappedSparkMax.create(
            null,
            null,
            new SmartMotorConfigBuilder()
                .setName("elevator")
                .setPort(elevatorMotorPort)
                .setReverseOutput(false)
                .setEnableBrakeMode(true)
                .setPdp(pdp)
                .setCurrentLimit(40)
                .setEnableVoltageComp(false)
                .build());
    // PID constants for elevator
    elevatorPulleyMotor.setPID(0, 0, 0);
    // WE ASSUME THE ELEVATOR STARTS AT THE BOTTOM
    // PLEASE MAKE SURE ELEVATOR IS ACTUALLY AT THE BOTTOM

    var elevator =
        new OneMotorPulleyElevator(
            elevatorPulleyMotor, ElevatorPosition.BOTTOM, elevatorMaxVelocity);
    var setVelocityCommand = new SetVelocity(elevator, mechanismsJoystick, elevatorMaxVelocity);

    // intake
    var intake =
        new IntakeActuated(
            new SolenoidSimple(new DoubleSolenoid(intakeSolenoidForward, intakeSolenoidReverse)));

    var subsystems = List.<Subsystem>of(drive, elevator, intake);

    var throttlePrototype =
        new ThrottlePolynomialBuilder().stick(driveJoystick).smoothingTimeSecs(0.04).scale(0.7);
    var rotThrottle =
        throttlePrototype
            .axis(0)
            .deadband(0.08)
            .inverted(false)
            .polynomial(new Polynomial(Map.of(1., 0.5), null))
            .build();
    var fwdThrottle =
        new ThrottleSum(
            new Throttle[] {
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
    var oi =
        new OIArcadeWithDPad(
            rotThrottle,
            fwdThrottle,
            0.1,
            false,
            driveJoystick,
            new Polynomial(
                Map.of(
                    0.5, 0.4,
                    0., 0.2),
                null),
            1.0,
            true);

    var intakeSolenoid =
        new SolenoidSimple(new DoubleSolenoid(intakeSolenoidForward, intakeSolenoidReverse));

    var updater = new Updater(List.of(pdp, drive, oi, navx));

    var defaultCommands = List.<DefaultCommand>of();

    var buttons =
        List.of(
            // Run bumper and transition wheel
            new CommandButton(
                new SimpleButton(driveJoystick, driverIntakeOutOn),
                new ParallelCommandGroup(
                    new SetSolenoidPose(intakeSolenoid, DoubleSolenoid.Value.kForward)
                    // todo more commands in this parallel command
                    ),
                CommandButton.Action.WHEN_PRESSED),
            // Shift drive up *or* down
            // todo Should there be 2 separate commands instead?
            new CommandButton(
                new SimpleButton(driveJoystick, shiftUp),
                new ShiftGears(drive),
                CommandButton.Action.WHEN_PRESSED),
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
            // elevator move to BOTTOM position
            new CommandButton(
                new SimpleButton(mechanismsJoystick, elevatorMoveToBottom),
                new MoveToPosition(ElevatorPosition.BOTTOM, elevator),
                CommandButton.Action.WHEN_PRESSED),
            // Close the intake
            new CommandButton(
                new SimpleButton(mechanismsJoystick, intakeClose),
                new SetIntake(IntakeActuated.IntakePosition.CLOSED, intake),
                CommandButton.Action.WHEN_PRESSED),
            // Open the intake
            new CommandButton(
                new SimpleButton(mechanismsJoystick, intakeOpen),
                new SetIntake(IntakeActuated.IntakePosition.OPEN, intake),
                CommandButton.Action.WHEN_PRESSED));

    var robotStartupCommands = List.<Command>of();
    var autoStartupCommands = List.<Command>of();
    var teleopStartupCommands = List.<Command>of(setVelocityCommand);
    var testStartupCommands = List.<Command>of();
    var allCommands =
        new CommandContainer(
            defaultCommands,
            buttons,
            robotStartupCommands,
            autoStartupCommands,
            teleopStartupCommands,
            testStartupCommands);

    return new RobotMap(subsystems, pdp, updater, allCommands, joysticks, useCameraServer);
  }
}
