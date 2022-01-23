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
import frc.team449.jacksonWrappers.MappedAHRS;
import frc.team449.jacksonWrappers.MappedJoystick;
import frc.team449.jacksonWrappers.PDP;
import frc.team449.jacksonWrappers.SlaveSparkMax;
import frc.team449.javaMaps.builders.DriveSettingsBuilder;
import frc.team449.javaMaps.builders.SparkMaxConfig;
import frc.team449.javaMaps.builders.ThrottlePolynomialBuilder;
import frc.team449.oi.throttles.Throttle;
import frc.team449.oi.throttles.ThrottleSum;
import frc.team449.oi.unidirectional.arcade.OIArcadeWithDPad;
import frc.team449.other.Debouncer;
import frc.team449.other.DefaultCommand;
import frc.team449.other.Updater;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Map;

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
            .setSlaveSparks(List.of(new SlaveSparkMax(RIGHT_LEADER_FOLLOWER_1_PORT, false)))
            .createReal();
    var leftMaster =
        driveMasterPrototype
            .copy()
            .setPort(LEFT_LEADER_PORT)
            .setName("left")
            .setReverseOutput(true)
            .setSlaveSparks(List.of(new SlaveSparkMax(LEFT_LEADER_FOLLOWER_1_PORT, false)))
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
            .polynomial(new Polynomial(Map.of(1., 0.009, 2., 0.002), null))
            .build();
    var fwdThrottle =
        new ThrottleSum(
            new Throttle[] {
              throttlePrototype
                  .axis(3)
                  .deadband(0.05)
                  .inverted(true)
                  .polynomial(
                      new Polynomial(
                          Map.of(
                              1., 0.01, // 0.06 * x^2 + 0.01 * x^1
                              2., 0.06),
                          null))
                  .build(),
              throttlePrototype.axis(2).inverted(false).build()
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
            0.7,
            true);

    var defaultDriveCommand =
        new DefaultCommand(
            drive,
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
