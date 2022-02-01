package frc.team449.javaMaps;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449._2022robot.cargo.Cargo2022;
import frc.team449._2022robot.climber.PivotingTelescopingClimber;
import frc.team449._2022robot.climber.commands.ExtendTelescopingArm;
import frc.team449._2022robot.climber.commands.RetractTelescopingArm;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.UnidirectionalNavXDefaultDrive;
import frc.team449.generalInterfaces.doubleUnaryOperator.Polynomial;
import frc.team449.generalInterfaces.doubleUnaryOperator.RampComponent;
import frc.team449.javaMaps.builders.DriveSettingsBuilder;
import frc.team449.javaMaps.builders.SparkMaxConfig;
import frc.team449.javaMaps.builders.ThrottlePolynomialBuilder;
import frc.team449.oi.buttons.SimpleButton;
import frc.team449.oi.throttles.ThrottlePolynomial;
import frc.team449.oi.throttles.ThrottleSum;
import frc.team449.oi.unidirectional.arcade.OIArcadeWithDPad;
import frc.team449.other.Debouncer;
import frc.team449.other.DefaultCommand;
import frc.team449.other.FollowerUtils;
import frc.team449.other.Updater;
import frc.team449.wrappers.AHRS;
import frc.team449.wrappers.PDP;
import frc.team449.wrappers.RumbleableJoystick;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Map;
import java.util.Set;

public class FullMap {
  // Motor IDs
  public static final int RIGHT_LEADER_PORT = 2,
      RIGHT_LEADER_FOLLOWER_1_PORT = 3,
      LEFT_LEADER_PORT = 1,
      LEFT_LEADER_FOLLOWER_1_PORT = 4,
      INTAKE_LEADER_PORT = 5,
      INTAKE_FOLLOWER_PORT = 6,
      SPITTER_PORT = 7,
      CLIMBER_MOTOR_PORT = 8;

  // Controller ports
  public static final int MECHANISMS_JOYSTICK_PORT = 0, DRIVE_JOYSTICK_PORT = 1;
  // Button numbers
  public static final int INTAKE_NORMAL_BUTTON = 1, INTAKE_REVERSE_BUTTON = 3, SPIT_BUTTON = 2;
  // Speeds
  public static final double INTAKE_SPEED = 0.1, SPITTER_SPEED = 0.1;

  private FullMap() {}

  @NotNull
  public static RobotMap createRobotMap() {

    var pdp = new PDP(1, new RunningLinRegComponent(250, 0.75), PowerDistribution.ModuleType.kCTRE);

    var mechanismsJoystick = new RumbleableJoystick(MECHANISMS_JOYSTICK_PORT);
    var driveJoystick = new RumbleableJoystick(DRIVE_JOYSTICK_PORT);
    List<GenericHID> joysticks = List.of(mechanismsJoystick, driveJoystick);

    var navx = new AHRS(SerialPort.Port.kMXP, true);

    var driveMasterPrototype =
        new SparkMaxConfig()
            .setEnableBrakeMode(true)
            .setUnitPerRotation(0.4787787204060999)
            .setCurrentLimit(50)
            .setEnableVoltageComp(true);
    var rightMaster =
        driveMasterPrototype
            .copy()
            .setName("right")
            .setPort(RIGHT_LEADER_PORT)
            .setReverseOutput(false)
            .addSlaveSpark(FollowerUtils.createFollowerSpark(RIGHT_LEADER_FOLLOWER_1_PORT), false)
            .createReal();
    var leftMaster =
        driveMasterPrototype
            .copy()
            .setPort(LEFT_LEADER_PORT)
            .setName("left")
            .setReverseOutput(true)
            .addSlaveSpark(FollowerUtils.createFollowerSpark(LEFT_LEADER_FOLLOWER_1_PORT), false)
            .createReal();

    var drive =
        new DriveUnidirectionalWithGyro(
            leftMaster,
            rightMaster,
            navx,
            new DriveSettingsBuilder()
                .postEncoderGearing(1 / 20.45)
                .maxSpeed(2.3)
                .leftFeedforward(new SimpleMotorFeedforward(0.24453, 5.4511, 0.7127))
                .rightFeedforward(new SimpleMotorFeedforward(0.2691, 5.3099, 0.51261))
                .build(),
            0.61755);

    var throttlePrototype =
        new ThrottlePolynomialBuilder().stick(driveJoystick).smoothingTimeSecs(0.04).scale(0.7);
    var rotThrottle =
        throttlePrototype
            .axis(0)
            .deadband(0.08)
            .inverted(false)
            .polynomial(
                new Polynomial(
                    Map.of(
                        1., 0.009,
                        2., 0.002),
                    null))
            .build();
    var fwdThrottle =
        new ThrottleSum(
            Set.of(
                throttlePrototype
                    .axis(3)
                    .deadband(0.05)
                    .inverted(true)
                    .polynomial(
                        new Polynomial(
                            Map.of(
                                1., 0.01,
                                2., 0.06),
                            null))
                    .build(),
                throttlePrototype.axis(2).inverted(false).build()));
    var oi =
        new OIArcadeWithDPad(
            rotThrottle,
            fwdThrottle,
            0.1,
            false,
            driveJoystick,
            new Polynomial(
                Map.of(
                    1., 1.), // Curvature
                null),
            0.7,
            true);

    drive.setDefaultCommand(
        new UnidirectionalNavXDefaultDrive<>(
            0,
            new Debouncer(1.5),
            0,
            1.0,
            null,
            2,
            3.0,
            false,
            0,
            0,
            0,
            new Debouncer(0.15),
            drive,
            oi,
            new RampComponent(2.0, 2.0)));

    var cargo =
        new Cargo2022(
            new SparkMaxConfig()
                .setName("intakeMotor")
                .setPort(INTAKE_LEADER_PORT)
                .addSlaveSpark(FollowerUtils.createFollowerSpark(INTAKE_FOLLOWER_PORT), false)
                .createReal(),
            new SparkMaxConfig().setName("spitterMotor").setPort(SPITTER_PORT).createReal(),
            INTAKE_SPEED,
            SPITTER_SPEED);

//    var climber =
//        new PivotingTelescopingClimber(
//            driveMasterPrototype
//                .copy()
//                .setName("climber_motor")
//                .setPort(CLIMBER_MOTOR_PORT)
//                .setUnitPerRotation(1)
//                .createReal(),
//            /*new SolenoidSimple(new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1))*/ null,
//            new DigitalInput(0),
//            new DigitalInput(1),
//            new ElevatorFeedforward(0, 0, 0, 0),
//            1,
//            0,
//            0,
//            5, // 1 rot/s max vel,
//            .5, // .5 rot/s^2
//            40 // rotations
//            );

    // PUT YOUR SUBSYSTEM IN HERE AFTER INITIALIZING IT
    var subsystems = List.<Subsystem>of(drive, cargo/**, climber*/);

    var updater = new Updater(List.of(pdp, navx));

    // Button bindings here

    // Take in balls but don't shoot
    new SimpleButton(mechanismsJoystick, INTAKE_NORMAL_BUTTON)
        .whileHeld(cargo::runIntake, cargo)
        .whenReleased(cargo::stop, cargo);
    // Run intake backwards so human can feed balls
    new SimpleButton(mechanismsJoystick, INTAKE_REVERSE_BUTTON)
        .whileHeld(cargo::runIntakeReverse, cargo)
        .whenReleased(cargo::stop, cargo);
    // Run all motors in intake to spit balls out
    new SimpleButton(mechanismsJoystick, SPIT_BUTTON)
        .whileHeld(cargo::spit, cargo)
        .whenReleased(cargo::stop, cargo);

    var defaultCommands = List.<DefaultCommand>of();

    // TODO BUTTON BINDINGS HERE
      /**
    new JoystickButton(mechanismsJoystick, XboxController.Button.kY.value)
        .whenPressed(new ExtendTelescopingArm(climber));
    new JoystickButton(mechanismsJoystick, XboxController.Button.kA.value)
        .whenPressed(new RetractTelescopingArm(climber));
        new JoystickButton(mechanismsJoystick, XboxController.Button.kX.value)
            .whenPressed(climber::pivotTelescopingArmIn, climber);
        new JoystickButton(mechanismsJoystick, XboxController.Button.kB.value)
            .whenPressed(climber::pivotTelescopingArmOut, climber);
     */

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
