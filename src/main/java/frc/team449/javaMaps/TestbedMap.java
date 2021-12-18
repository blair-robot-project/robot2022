package frc.team449.javaMaps;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.components.ShiftComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyroShiftable;
import frc.team449.generalInterfaces.SmartMotor;
import frc.team449.generalInterfaces.shiftable.Shiftable;
import frc.team449.jacksonWrappers.FeedForwardCalculators.MappedFeedForwardCalculator;
import frc.team449.jacksonWrappers.*;
import frc.team449.javaMaps.builders.PerGearSettingsBuilder;
import frc.team449.javaMaps.builders.SmartMotorConfig;
import frc.team449.oi.buttons.CommandButton;
import frc.team449.other.DefaultCommand;
import frc.team449.other.Updater;
import org.jetbrains.annotations.NotNull;

import java.util.List;

public class TestbedMap {
    // Motor IDs
    public static final int RIGHT_LEADER_PORT = 2,
            RIGHT_LEADER_FOLLOWER_1_PORT = 3,
            LEFT_LEADER_PORT = 1,
            LEFT_LEADER_FOLLOWER_1_PORT = 4;
    // Solenoid ports
    public static final int INTAKE_SOLENOID_FORWARD_PORT = 2, INTAKE_SOLENOID_REVERSE_PORT = 3;
    // Controller ports
    public static final int MECHANISMS_JOYSTICK_PORT = 0, DRIVE_JOYSTICK_PORT = 1;
    // Drive button numbers
    public static final int SHIFT_TOGGLE_BUTTON = 5;

    private TestbedMap() {}

    @NotNull
    public static RobotMap createRobotMap() {
        var useCameraServer = false;
        var pdp = new PDP(0, new RunningLinRegComponent(250, 0.75));

        var driveJoystick = new MappedJoystick(DRIVE_JOYSTICK_PORT);
        var joysticks = List.of(driveJoystick);

        var compressor = new Compressor();
        compressor.start();
        var gearShiftingSolenoids = new DoubleSolenoid(0, 1, 0);

        var navx = new MappedAHRS(SerialPort.Port.kMXP, true);
        var driveMasterPrototype =
                new SmartMotorConfig()
                        .setType(SmartMotor.Type.SPARK)
                        .setEnableBrakeMode(true)
                        .setPdp(pdp)
                        .setUnitPerRotation(0.4787787204060999)
                        .setCurrentLimit(50)
                        .setEnableVoltageComp(true)
                        .setStartingGear(Shiftable.Gear.HIGH);
        var lowGear =
                new PerGearSettingsBuilder()
                        .gear(Shiftable.Gear.LOW)
                        .postEncoderGearing(1 / 20.45)
                        .maxSpeed(2.3);
        var highGear =
                new PerGearSettingsBuilder()
                        .gear(Shiftable.Gear.HIGH)
                        .postEncoderGearing(1 / 7.73)
                        .maxSpeed(5.2); // free speed max in m/s is 44.537592495 m/s
        var rightMaster =
                new MappedTalon(
                        null,
                        null,
                        null,
                        null,
                        null,
                        null,
                        null,
                        null,
                        null,
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
                                                                new MappedFeedForwardCalculator(0.2691, 5.3099, 0.51261))
                                                        .build(),
                                                highGear
                                                        .feedForwardCalculator(
                                                                new MappedFeedForwardCalculator(
                                                                        0.165, 2.01, 0.155)) // TODO characterize
                                                        .build()))
                                .ensureBuilt());
        var leftMaster =
                new MappedTalon(
                        null,
                        null,
                        null,
                        null,
                        null,
                        null,
                        null,
                        null,
                        null,
                        driveMasterPrototype
                                .setPort(LEFT_LEADER_PORT)
                                .setName("left")
                                .setReverseOutput(true)
                                .setSlaveSparks(List.of(new SlaveSparkMax(LEFT_LEADER_FOLLOWER_1_PORT, false, pdp)))
                                .setPerGearSettings(
                                        List.of(
                                                lowGear
                                                        .feedForwardCalculator(
                                                                new MappedFeedForwardCalculator(0.24453, 5.4511, 0.7127))
                                                        .build(),
                                                highGear
                                                        .feedForwardCalculator(
                                                                new MappedFeedForwardCalculator(
                                                                        0.156, 2.01, 0.154)) // TODO characterize
                                                        .build()))
                                .ensureBuilt());

        var drive =
                new DriveUnidirectionalWithGyroShiftable(
                        leftMaster,
                        rightMaster,
                        navx,
                        0.61755,
                        new ShiftComponent(
                                List.of(leftMaster, rightMaster), gearShiftingSolenoids, Shiftable.Gear.LOW),
                        false);

        var subsystems = List.<Subsystem>of(drive);

        var updater = new Updater(List.of(pdp, navx, drive));

        var defaultCommands = List.<DefaultCommand>of();

        var buttons =
                List.<CommandButton>of(
                        //            // toggle shift gears
                        //            new CommandButton(
                        //                new SimpleButton(driveJoystick, SHIFT_TOGGLE_BUTTON),
                        //                new ShiftGears(drive),
                        //                CommandButton.Action.WHEN_PRESSED)
                );

        var robotStartupCommands = List.<Command>of();
        var autoStartupCommands = List.<Command>of(
//                new RamseteControllerGoToPosition(
//
//                )
        );
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
