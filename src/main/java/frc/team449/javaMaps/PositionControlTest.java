package frc.team449.javaMaps;

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
import frc.team449.javaMaps.builders.SmartMotorConfigBuilder;
import frc.team449.oi.buttons.CommandButton;
import frc.team449.oi.buttons.SimpleButton;
import frc.team449.other.DefaultCommand;
import frc.team449.other.Updater;
import java.util.List;
import org.jetbrains.annotations.NotNull;

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
    int leftMasterPort = 1,
        leftMasterSlave1Port = 3,
        leftMasterSlave2Port = 5,
        rightMasterPort = 2,
        rightMasterSlave1Port = 4,
        rightMasterSlave2Port = 6,
        elevatorMotorPort = 9;

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
    double elevatorMaxVelocity = 5; // TODO this is a placeholder

    var useCameraServer = false;
    var pdp = new PDP(0, new RunningLinRegComponent(250, 0.75));

    var mechanismsJoystick = new MappedJoystick(mechanismsJoystickPort);
    //        var driveJoystick = new MappedJoystick(driveJoystickPort);
    var joysticks = List.of(mechanismsJoystick /*, driveJoystick*/);
    //
    //        var navx = new MappedAHRS(SerialPort.Port.kMXP, true);
    //        var driveMasterPrototype =
    //                new SmartMotorConfigObject()
    //                        .setType(SmartMotor.Type.SPARK)
    //                        .setEnableBrakeMode(true)
    //                        .setPdp(pdp)
    //                        .setUnitPerRotation(0.47877872)
    //                        .setCurrentLimit(50)
    //                        .setEnableVoltageComp(true)
    //                        .setStartingGear(Shiftable.Gear.LOW)
    //                        .setEncoderCPR(256);
    //        var lowGear =
    //                new PerGearSettingsBuilder()
    //                        .gear(Shiftable.Gear.LOW)
    //                        .postEncoderGearing(0.0488998)
    //                        .maxSpeed(2.3)
    //                        .kP(0);
    //        var highGear =
    //                new PerGearSettingsBuilder()
    //                        .gear(Shiftable.Gear.HIGH)
    //                        .postEncoderGearing(0.12936611)
    //                        .maxSpeed(5.2)
    //                        .kP(0.000001);
    //
    //        var leftMaster =
    //                SmartMotor.create(
    //                        driveMasterPrototype
    //                                .setPort(leftMasterPort)
    //                                .setName("left")
    //                                .setReverseOutput(true)
    //                                .setSlaveSparks(
    //                                        List.of(
    //                                                new SlaveSparkMax(leftMasterSlave1Port, false,
    // pdp),
    //                                                new SlaveSparkMax(leftMasterSlave2Port, false,
    // pdp)))
    //                                .setPerGearSettings(
    //                                        List.of(
    //                                                lowGear
    //                                                        .feedForwardCalculator(
    //                                                                new
    // MappedFeedForwardCalculator(0.128, 5.23, 0.0698))
    //                                                        .build(),
    //                                                highGear
    //                                                        .feedForwardCalculator(
    //                                                                new
    // MappedFeedForwardCalculator(0.156, 2.01, 0.154))
    //                                                        .build())));
    //        var rightMaster =
    //                SmartMotor.create(
    //                        driveMasterPrototype
    //                                .setName("right")
    //                                .setPort(rightMasterPort)
    //                                .setReverseOutput(false)
    //                                .setSlaveSparks(
    //                                        List.of(
    //                                                new SlaveSparkMax(rightMasterSlave1Port,
    // false, pdp),
    //                                                new SlaveSparkMax(rightMasterSlave2Port,
    // false, pdp)))
    //                                .setPerGearSettings(
    //                                        List.of(
    //                                                lowGear
    //                                                        .feedForwardCalculator(
    //                                                                new
    // MappedFeedForwardCalculator(0.139, 5.17, 0.0554))
    //                                                        .build(),
    //                                                highGear
    //                                                        .feedForwardCalculator(
    //                                                                new
    // MappedFeedForwardCalculator(0.165, 2.01, 0.155))
    //                                                        .build())));
    /*var drive =
        new DriveUnidirectionalWithGyroShiftable(
                leftMaster,
                rightMaster,
                navx,
                0.61755,
                new ShiftComponent(
                        List.of(leftMaster, rightMaster), new DoubleSolenoid(0, 0, 1), Shiftable.Gear.LOW),
                false);
    */
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
                .setPerGearSettings(List.of(
                    new PerGearSettingsBuilder()
                        .gear(Shiftable.Gear.LOW)
                        .maxSpeed(elevatorMaxVelocity)
                        .build()))
                .build());
    // PID constants for velocity controlled elevator motor
    //    elevatorPulleyMotor.setPID(0.0003, 0.0000008, 0.0146);
    // PID constants for position controlled elevator motor
    elevatorPulleyMotor.setPID(.045, .00000095, 1);
    // WE ASSUME THE ELEVATOR STARTS AT THE BOTTOM
    // PLEASE MAKE SURE ELEVATOR IS ACTUALLY AT THE BOTTOM

    var elevator = new OneMotorPulleyElevator(elevatorPulleyMotor, ElevatorPosition.BOTTOM);
    var setVelocityCommand = new SetVelocity(elevator, mechanismsJoystick, elevatorMaxVelocity);

    // intake
    /*var intake =
                new IntakeActuated(
                        new SolenoidSimple(new DoubleSolenoid(intakeSolenoidForward, intakeSolenoidReverse)));
    */
    var subsystems = List.<Subsystem>of(elevator);

    //        var throttlePrototype =
    //                new
    // ThrottlePolynomialBuilder().stick(driveJoystick).smoothingTimeSecs(0.04).scale(0.7);
    //        var rotThrottle =
    //                throttlePrototype
    //                        .axis(0)
    //                        .deadband(0.08)
    //                        .inverted(false)
    //                        .polynomial(new Polynomial(Map.of(1., 0.5), null))
    //                        .build();
    //        var fwdThrottle =
    //                new ThrottleSum(
    //                        new Throttle[] {
    //                                throttlePrototype
    //                                        .axis(3)
    //                                        .deadband(0.05)
    //                                        .inverted(false)
    //                                        .polynomial(
    //                                                new Polynomial(
    //                                                        Map.of(
    //                                                                1., 2.,
    //                                                                2., 1.),
    //                                                        null))
    //                                        .build(),
    //                                throttlePrototype.axis(2).inverted(true).build()
    //                        });
    //        var oi =
    //                new OIArcadeWithDPad(
    //                        rotThrottle,
    //                        fwdThrottle,
    //                        0.1,
    //                        false,
    //                        driveJoystick,
    //                        new Polynomial(
    //                                Map.of(
    //                                        0.5, 0.4,
    //                                        0., 0.2),
    //                                null),
    //                        1.0,
    //                        true);
    ////
    //        var intakeSolenoid =
    //                new SolenoidSimple(new DoubleSolenoid(intakeSolenoidForward,
    // intakeSolenoidReverse));

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

    return new RobotMap(subsystems, pdp, updater, allCommands, joysticks, useCameraServer);
  }
}
