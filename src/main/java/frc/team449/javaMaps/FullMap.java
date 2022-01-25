package frc.team449.javaMaps;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.DriveAtSpeed;
import frc.team449.drive.unidirectional.commands.UnidirectionalNavXDefaultDrive;
import frc.team449.generalInterfaces.doubleUnaryOperator.Polynomial;
import frc.team449.generalInterfaces.doubleUnaryOperator.RampComponent;
import frc.team449.jacksonWrappers.*;
import frc.team449.javaMaps.builders.DriveSettingsBuilder;
import frc.team449.javaMaps.builders.SparkMaxConfig;
import frc.team449.javaMaps.builders.TalonConfig;
import frc.team449.javaMaps.builders.ThrottlePolynomialBuilder;
import frc.team449.oi.throttles.ThrottleSum;
import frc.team449.oi.unidirectional.arcade.OIArcadeWithDPad;
import frc.team449.other.Debouncer;
import frc.team449.other.DefaultCommand;
import frc.team449.other.Updater;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Map;
import java.util.Set;

import static com.ctre.phoenix.motorcontrol.InvertType.InvertMotorOutput;
import static com.ctre.phoenix.motorcontrol.InvertType.None;

public class FullMap {
  // Motor IDs
  public static final int RIGHT_LEADER_PORT = 4,
      RIGHT_LEADER_FOLLOWER_1_PORT = 5,
      RIGHT_LEADER_FOLLOWER_2_PORT = 6,
      LEFT_LEADER_PORT = 1,
      LEFT_LEADER_FOLLOWER_1_PORT = 2,
      LEFT_LEADER_FOLLOWER_2_PORT = 3;
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

    var driveMasterPrototype =
        new TalonConfig()
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
            .addSlaveTalon(new SlaveTalon(RIGHT_LEADER_FOLLOWER_1_PORT, None))
            .addSlaveTalon(new SlaveTalon(RIGHT_LEADER_FOLLOWER_2_PORT, None))
            .createReal();
    var leftMaster =
        driveMasterPrototype
            .copy()
            .setPort(LEFT_LEADER_PORT)
            .setName("left")
            .setReverseOutput(true)
            .addSlaveTalon(new SlaveTalon(LEFT_LEADER_FOLLOWER_1_PORT, InvertMotorOutput))
            .addSlaveTalon(new SlaveTalon(LEFT_LEADER_FOLLOWER_2_PORT, InvertMotorOutput))
            .createReal();

    var drive =
        new DriveUnidirectionalWithGyro(
            leftMaster,
            rightMaster,
            navx,
            new DriveSettingsBuilder()
                .postEncoderGearing(1 / 20.45)
                .maxSpeed(2.3)
                .leftFeedforward(new SimpleMotorFeedforward(0.797588, 1.32498443423, 0.12919953323))
                .rightFeedforward(new SimpleMotorFeedforward(0.2691, 5.3099, 0.51261))
                .build(),
            0.63);

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
                        1., 1.,
                        3., 3.),
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
                                1., 2.,
                                3., 1.),
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

    var defaultDriveCommand =
        new DefaultCommand(
            drive,
            new UnidirectionalNavXDefaultDrive<>(
                0,
                new Debouncer(.15),
                0,
                1.0,
                null,
                2,
                15.0,
                false,
                0.0075,
                0,
                0.03,
                new Debouncer(0.15),
                drive,
                oi,
                new RampComponent(3.0, 3.0)));

    var subsystems =
        List.<Subsystem>of(drive); // TODO PUT YOUR SUBSYSTEM IN HERE AFTER INITIALIZING IT

    var updater = new Updater(List.of(pdp, oi, navx, drive));

    var defaultCommands = List.of(defaultDriveCommand);

    // TODO BUTTON BINDINGS HERE

    List<Command> robotStartupCommands = List.of();

    List<Command> autoStartupCommands =
        List.of(
            // todo tune this and ultimately replace with a more sophisticated command
            new DriveAtSpeed<>(drive, 0.1, 1.5));

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
