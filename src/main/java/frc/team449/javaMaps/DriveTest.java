package frc.team449.javaMaps;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.components.ShiftComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyroShiftable;
import frc.team449.generalInterfaces.SmartMotor;
import frc.team449.generalInterfaces.doubleUnaryOperator.Polynomial;
import frc.team449.generalInterfaces.shiftable.Shiftable;
import frc.team449.jacksonWrappers.*;
import frc.team449.jacksonWrappers.FeedForwardCalculators.MappedFeedForwardCalculator;
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
import java.util.List;
import java.util.Map;
import org.jetbrains.annotations.NotNull;

public class DriveTest {
  // Drive system
  public static final int RIGHT_MASTER_PORT = 1,
      RIGHT_MASTER_SLAVE_1_PORT = 2,
      RIGHT_MASTER_SLAVE_2_PORT = 3,
      LEFT_MASTER_PORT = 4,
      LEFT_MASTER_SLAVE_1_PORT = 5,
      LEFT_MASTER_SLAVE_2_PORT = 6;
  // Solenoid ports
  public static final int INTAKE_SOLENOID_FORWARD_PORT = 2, INTAKE_SOLENOID_REVERSE_PORT = 3;
  public static final int MECHANISMS_JOYSTICK_PORT = 0, DRIVE_JOYSTICK_PORT = 1;

  private DriveTest() {
    throw new IllegalStateException("This is a utility class!");
  }

  @NotNull
  public static RobotMap createRobotMap() {
    // TODO Declare these constants outside this method and remove unused variables

    // Motor ports
    int rightMasterPort = 1,
        rightMasterSlave1Port = 2,
        rightMasterSlave2Port = 3,
        leftMasterPort = 4,
        leftMasterSlave1Port = 5,
        leftMasterSlave2Port = 6,
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
        elevatorMoveToBottom = 4;
    // Motor speeds

    var useCameraServer = false;
    var pdp = new PDP(0, new RunningLinRegComponent(250, 0.75));

//    var mechanismsJoystick = new MappedJoystick(mechanismsJoystickPort);
    var driveJoystick = new MappedJoystick(driveJoystickPort);
    var joysticks = List.of(/*mechanismsJoystick,*/ driveJoystick);

    var compressor = new Compressor();

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
    var lowGear =
        new PerGearSettingsBuilder()
            .gear(Shiftable.Gear.LOW)
            .postEncoderGearing(0.0488998)
            .maxSpeed(2.3);
    var highGear =
        new PerGearSettingsBuilder()
            .gear(Shiftable.Gear.HIGH)
            .postEncoderGearing(0.12936611)
            .maxSpeed(5.2);
    var rightMaster =
        SmartMotor.create(
            driveMasterPrototype
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
                            .build())));
    var leftMaster =
        SmartMotor.create(
            driveMasterPrototype
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
                            .build())));

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
    //        var elevatorPulleyMotor =
    //                new MappedSparkMax(
    //                        elevatorMotorPort,
    //                        "elevator",
    //                        false,
    //                        true,
    //                        pdp,
    //                        null,
    //                        null,
    //                        null,
    //                        null,
    //                        null,
    //                        1.0 / 30.0,
    //                        1.0,
    //                        40,
    //                        false,
    //                        List.of(new PerGearSettingsBuilder().
    //                                gear(Shiftable.Gear.LOW).feedForwardCalculator(
    //                                        new MappedFeedForwardCalculator(0.0993, 0.00414,
    // 0.000128))
    //                                .maxSpeed(5000.0)
    //                                .build()),
    //                        Shiftable.Gear.LOW,
    //                        null,
    //                        null,
    //                        null,
    //                        null);
    //        // PID constants for velocity controlled elevator motor
    ////    elevatorPulleyMotor.setPID(0.0003, 0.0000008, 0.0146);
    //        // PID constants for position controlled elevator motor
    //        elevatorPulleyMotor.setPID(1,0,1);
    //        // WE ASSUME THE ELEVATOR STARTS AT THE BOTTOM
    //        // PLEASE MAKE SURE ELEVATOR IS ACTUALLY AT THE BOTTOM
    //
    //        var elevator = new OneMotorPulleyElevator(elevatorPulleyMotor,
    // ElevatorPosition.BOTTOM);
    //        var setVelocityCommand = new SetVelocity(elevator, mechanismsJoystick,
    // elevatorMaxVelocity);

    // intake
    /*var intake =
                new IntakeActuated(
                        new SolenoidSimple(new DoubleSolenoid(intakeSolenoidForward, intakeSolenoidReverse)));
    */
    var subsystems = List.<Subsystem>of(drive);

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
    //
    //        var intakeSolenoid =
    //                new SolenoidSimple(new DoubleSolenoid(intakeSolenoidForward,
    // intakeSolenoidReverse));

    var updater = new Updater(List.of(pdp, oi, navx, drive));

    var defaultCommands = List.<DefaultCommand>of();

    var buttons = List.<CommandButton>of(
            // start left side
            new CommandButton(
                    new SimpleButton(driveJoystick, 1),
                    new InstantCommand(() -> leftMaster.setVelocity(.2),drive),
                    CommandButton.Action.WHEN_PRESSED
            ),
            // start right side
            new CommandButton(
                    new SimpleButton(driveJoystick, 4),
                    new InstantCommand(() -> rightMaster.setVelocity(.2),drive),
                    CommandButton.Action.WHEN_PRESSED
            ),
            // stop left side
            new CommandButton(
                    new SimpleButton(driveJoystick, 2),
                    new InstantCommand(() -> leftMaster.setVelocity(0),drive),
                    CommandButton.Action.WHEN_PRESSED
            ),
            // stop right side
            new CommandButton(
                    new SimpleButton(driveJoystick,3),
                    new InstantCommand(() -> rightMaster.setVelocity(0), drive),
                    CommandButton.Action.WHEN_PRESSED
            )
    );

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
