package frc.team449.javaMaps;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449._2022robot.cargo.Cargo2022;
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
import frc.team449.oi.buttons.SimpleButton;
import frc.team449.oi.throttles.ThrottleSum;
import frc.team449.oi.unidirectional.arcade.OIArcadeWithDPad;
import frc.team449.other.Debouncer;
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
      LEFT_LEADER_FOLLOWER_1_PORT = 4,
      INTAKE_LEADER_PORT = 5,
      INTAKE_FOLLOWER_PORT = 6,
      SPITTER_PORT = 7;
  // Controller ports
  public static final int MECHANISMS_JOYSTICK_PORT = 0, DRIVE_JOYSTICK_PORT = 1;
  // Button numbers
  public static final int INTAKE_NORMAL_BUTTON = 1,
      INTAKE_REVERSE_BUTTON = 3,
      SPIT_BUTTON = 2;
  // Speeds
  public static final double INTAKE_SPEED = 0.1, SPITTER_SPEED = 0.1;

  private FullMap() {}

  @NotNull
  public static RobotMap createRobotMap() {

    var pdp = new PDP(1, new RunningLinRegComponent(250, 0.75), PowerDistribution.ModuleType.kCTRE);

    var mechanismsJoystick = new MappedJoystick(MECHANISMS_JOYSTICK_PORT);
    var driveJoystick = new MappedJoystick(DRIVE_JOYSTICK_PORT);
    List<GenericHID> joysticks = List.of(mechanismsJoystick, driveJoystick);

    var navx = new MappedAHRS(SerialPort.Port.kMXP, true);

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
            .addSlaveSpark(new SlaveSparkMax(RIGHT_LEADER_FOLLOWER_1_PORT, false))
            .createReal();
    var leftMaster =
        driveMasterPrototype
            .copy()
            .setPort(LEFT_LEADER_PORT)
            .setName("left")
            .setReverseOutput(true)
            .addSlaveSpark(new SlaveSparkMax(LEFT_LEADER_FOLLOWER_1_PORT, false))
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
                    0.5, 0.4,
                    0., 0.2),
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
                .addSlaveSpark(new SlaveSparkMax(INTAKE_FOLLOWER_PORT, false))
                .createReal(),
            new SparkMaxConfig().setName("spitterMotor").setPort(SPITTER_PORT).createReal(),
            INTAKE_SPEED,
            SPITTER_SPEED);

    // TODO PUT YOUR SUBSYSTEM IN HERE AFTER INITIALIZING IT
    List<Subsystem> subsystems = List.of(drive, cargo);

    var updater = new Updater(List.of(pdp, oi, navx, drive));

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

    List<Command> robotStartupCommands = List.of();

    List<Command> autoStartupCommands =
        List.of(
            // todo tune this and ultimately replace with a more sophisticated command
            new DriveAtSpeed<>(drive, 0.1, 1.5));

    List<Command> teleopStartupCommands = List.of();

    List<Command> testStartupCommands = List.of();
    var allCommands =
        new CommandContainer(
            robotStartupCommands,
            autoStartupCommands,
            teleopStartupCommands,
            testStartupCommands);

    return new RobotMap(subsystems, pdp, updater, allCommands, joysticks, false);
  }
}
