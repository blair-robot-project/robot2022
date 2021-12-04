package frc.team449.javaMaps;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449._2020.multiSubsystem.SolenoidSimple;
import frc.team449._2021BunnyBot.elevator.OneMotorPulleyElevator;
import frc.team449._2021BunnyBot.elevator.commands.MoveToPosition;
import frc.team449._2021BunnyBot.intake.OnePistonIntake;
import frc.team449._2021BunnyBot.intake.commands.SetIntake;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.components.ShiftComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyroShiftable;
import frc.team449.drive.unidirectional.commands.UnidirectionalNavXDefaultDrive;
import frc.team449.generalInterfaces.SmartMotor;
import frc.team449.generalInterfaces.doubleUnaryOperator.Polynomial;
import frc.team449.generalInterfaces.doubleUnaryOperator.RampComponent;
import frc.team449.generalInterfaces.shiftable.Shiftable;
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
import frc.team449.other.Debouncer;
import frc.team449.other.DefaultCommand;
import frc.team449.other.Updater;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Map;

public class Bunnybot2021Map {
  // Motor IDs
  public static final int RIGHT_LEADER_PORT = 1;
  public static final int RIGHT_LEADER_FOLLOWER_1_PORT = 2;
  public static final int LEFT_LEADER_PORT = 3;
  public static final int LEFT_LEADER_FOLLOWER_1_PORT = 4;
  // Solenoid ports
  public static final int INTAKE_SOLENOID_FORWARD_PORT = 2;
  public static final int INTAKE_SOLENOID_REVERSE_PORT = 3;
  // Controller ports
  public static final int MECHANISMS_JOYSTICK_PORT = 0;
  public static final int DRIVE_JOYSTICK_PORT = 1;
  // Drive button numbers
  public static final int SHIFT_TOGGLE_BUTTON = 5;
  // Elevator stuff
  private static final double ELEVATOR_MAX_VELOCITY = 5; // TODO this is a placeholder
  private static final int ELEVATOR_MOTOR_PORT = 9;
  private static final int ELEVATOR_MOVE_TO_TOP = 1;
  private static final int ELEVATOR_MOVE_TO_UPPER = 2;
  private static final int ELEVATOR_MOVE_TO_LOWER = 3;
  private static final int ELEVATOR_MOVE_TO_BOTTOM = 4;
  // Intake stuff
  private static final int INTAKE_CLOSE = 7;
  private static final int INTAKE_OPEN = 8;

  private Bunnybot2021Map() {
    throw new IllegalStateException("This is a utility class!");
  }

  @NotNull
  public static RobotMap createRobotMap() {
    var useCameraServer = false;
    var pdp = new PDP(0, new RunningLinRegComponent(250, 0.75));

    var driveJoystick = new MappedJoystick(DRIVE_JOYSTICK_PORT);
    var mechanismsJoystick = new MappedJoystick(MECHANISMS_JOYSTICK_PORT);
    var joysticks = List.of(driveJoystick, mechanismsJoystick);

    var compressor = new Compressor();
    var gearShiftingSolenoids = new DoubleSolenoid(0, 1, 0);

    var navx = new MappedAHRS(SerialPort.Port.kMXP, true);
    var driveMasterPrototype =
        new SmartMotorConfigObject()
            .setType(SmartMotor.Type.SPARK)
            .setEnableBrakeMode(true)
            .setPdp(pdp)
            .setUnitPerRotation(0.470799075)
            .setCurrentLimit(50)
            .setEnableVoltageComp(true)
            .setStartingGear(Shiftable.Gear.HIGH);
    var lowGear =
        new PerGearSettingsBuilder()
            .gear(Shiftable.Gear.LOW)
            .postEncoderGearing(1 / 20.45)
            .maxSpeed(1.0); // in m/s
    var highGear =
        new PerGearSettingsBuilder()
            .gear(Shiftable.Gear.HIGH)
            .postEncoderGearing(1 / 7.73)
            .maxSpeed(2.0); // in m/s
    var rightMaster =
        SmartMotor.create(
            driveMasterPrototype
                .setName("right")
                .setPort(RIGHT_LEADER_PORT)
                .setReverseOutput(false)
                .setSlaveSparks(
                    List.of(new SlaveSparkMax(RIGHT_LEADER_FOLLOWER_1_PORT, false, pdp)))
                .setPerGearSettings(
                    List.of(
                        lowGear
                            .feedForwardCalculator(
                                new MappedFeedForwardCalculator(0.102, 5.66, 0.306)) // TODO characterize
                            .build(),
                        highGear
                            .feedForwardCalculator(
                                new MappedFeedForwardCalculator(
                                    0.165, 2.01, 0.155)) // TODO characterize
                            .build())));
    var leftMaster =
        SmartMotor.create(
            driveMasterPrototype
                .setPort(LEFT_LEADER_PORT)
                .setName("left")
                .setReverseOutput(true)
                .setSlaveSparks(List.of(new SlaveSparkMax(LEFT_LEADER_FOLLOWER_1_PORT, false, pdp)))
                .setPerGearSettings(
                    List.of(
                        lowGear
                            .feedForwardCalculator(
                                new MappedFeedForwardCalculator(0.102, 5.66, 0.306)) //TODO characterize
                            .build(),
                        highGear
                            .feedForwardCalculator(
                                new MappedFeedForwardCalculator(
                                    0.156, 2.01, 0.154)) // TODO characterize
                            .build())));

    var drive =
        new DriveUnidirectionalWithGyroShiftable(
            leftMaster,
            rightMaster,
            navx,
            0.61755, //TODO measure the distance from one side of the drive to the other
            new ShiftComponent(
                List.of(leftMaster, rightMaster), gearShiftingSolenoids, Shiftable.Gear.LOW),
            false);

    // Elevator
    // TODO setup elevator, with Meters
    var elevatorPulleyMotor =
        new MappedSparkMax(
            ELEVATOR_MOTOR_PORT,
            "elevator",
            false,
            true,
            pdp,
            null,
            null,
            null,
            null,
            null,
            1.0,
            1.0, //TODO set to circumference of pulley for the elevator
            40,
            false,
            List.of(new PerGearSettingsBuilder().gear(Shiftable.Gear.LOW).maxSpeed(2.0).build()),
            Shiftable.Gear.LOW,
            null,
            null,
            null,
            null);
    // PID constants for position controlled elevator motor
    elevatorPulleyMotor.setPID(0, 0, 0); // TODO tune pid
    // WE ASSUME THE ELEVATOR STARTS AT THE BOTTOM
    // PLEASE MAKE SURE ELEVATOR IS ACTUALLY AT THE BOTTOM

    var elevator = new OneMotorPulleyElevator(
            elevatorPulleyMotor, OneMotorPulleyElevator.ElevatorPosition.BOTTOM);

    // Intake
    var intake =
        new OnePistonIntake(
            new SolenoidSimple(
                new DoubleSolenoid(INTAKE_SOLENOID_FORWARD_PORT, INTAKE_SOLENOID_REVERSE_PORT)
            )
        );
    // Controls for the drive
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

    var defaultDriveCommand =
        new DefaultCommand(
            drive,
            new UnidirectionalNavXDefaultDrive<DriveUnidirectionalWithGyro>(
                0,
                new Debouncer(1.5),
                0,
                1.0,
                null,
                2,
                3.0,
                false,
                0, // TODO tune pid
                0,
                0,
                new Debouncer(0.15),
                drive,
                oi,
                new RampComponent(3.0, 3.0)
            )
        );

    var subsystems = List.<Subsystem>of(drive, elevator, intake);

    var updater = new Updater(List.of(pdp, oi, navx, drive));

    var defaultCommands = List.of(defaultDriveCommand);

    var buttons =
        List.of(
            // elevator move to TOP position
            new CommandButton(
                new SimpleButton(mechanismsJoystick, ELEVATOR_MOVE_TO_TOP),
                new MoveToPosition(OneMotorPulleyElevator.ElevatorPosition.TOP, elevator),
                CommandButton.Action.WHEN_PRESSED),
            // elevator move to UPPER position
            new CommandButton(
                new SimpleButton(mechanismsJoystick, ELEVATOR_MOVE_TO_UPPER),
                new MoveToPosition(OneMotorPulleyElevator.ElevatorPosition.UPPER, elevator),
                CommandButton.Action.WHEN_PRESSED),
            // elevator move to LOWER position
            new CommandButton(
                new SimpleButton(mechanismsJoystick, ELEVATOR_MOVE_TO_LOWER),
                new MoveToPosition(OneMotorPulleyElevator.ElevatorPosition.LOWER, elevator),
                CommandButton.Action.WHEN_PRESSED),
            // elevator move to BOTTOM position
            new CommandButton(
                new SimpleButton(mechanismsJoystick, ELEVATOR_MOVE_TO_BOTTOM),
                new MoveToPosition(OneMotorPulleyElevator.ElevatorPosition.BOTTOM, elevator),
                CommandButton.Action.WHEN_PRESSED),
            // Close the intake
            new CommandButton(
                new SimpleButton(mechanismsJoystick, INTAKE_CLOSE),
                new SetIntake(OnePistonIntake.IntakePosition.CLOSED, intake),
                CommandButton.Action.WHEN_PRESSED),
            // Open the intake
            new CommandButton(
                new SimpleButton(mechanismsJoystick, INTAKE_OPEN),
                new SetIntake(OnePistonIntake.IntakePosition.OPEN, intake),
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
