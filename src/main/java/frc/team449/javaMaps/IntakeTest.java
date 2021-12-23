package frc.team449.javaMaps;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.jacksonWrappers.MappedJoystick;
import frc.team449.jacksonWrappers.PDP;
import frc.team449.oi.buttons.CommandButton;
import frc.team449.oi.buttons.SimpleButton;
import frc.team449.other.DefaultCommand;
import frc.team449.other.Updater;
import org.jetbrains.annotations.NotNull;

import java.util.List;

public class IntakeTest {
  // Solenoid ports
  // todo check these channels
  private static final int INTAKE_SOLENOID_FORWARD_CHANNEL = 0, INTAKE_SOLENOID_REVERSE_CHANNEL = 1;

  // Joystick ports
  private static final int MECHANISMS_JOYSTICK_PORT = 0;

  // Mechs button numbers
  private static final int INTAKE_OPEN_BUTTON = 1, INTAKE_CLOSE_BUTTON = 2;

  private static final boolean USE_CAMERA_SERVER = false;

  @NotNull
  public static RobotMap createRobotMap() {
    var pdp = new PDP(0, new RunningLinRegComponent(250, 0.75));

    var mechanismsJoystick = new MappedJoystick(MECHANISMS_JOYSTICK_PORT);
    var joysticks = List.of(mechanismsJoystick);

    // intake
    // todo make a DoubleSolenoidSubsystem class in jacksonWrappers or someplace else instead of the
    // _2020 package
    //  jacksonWrappers itself should be renamed, but that's an issue for another day
    var intake =
        new DoubleSolenoid(INTAKE_SOLENOID_FORWARD_CHANNEL, INTAKE_SOLENOID_REVERSE_CHANNEL);
    //    var intake2 = new Solenoid(INTAKE_SOLENOID_FORWARD_CHANNEL);
    var compressor = new Compressor(0);
    compressor.start();

    var subsystems = List.<Subsystem>of();
    var updater = new Updater(List.of(pdp));

    var buttons =
        List.of(
            // Close intake
            new CommandButton(
                new SimpleButton(mechanismsJoystick, INTAKE_CLOSE_BUTTON),
                new InstantCommand(() -> intake.set(DoubleSolenoid.Value.kForward)),
                CommandButton.Action.WHEN_PRESSED),
            // Open intake
            new CommandButton(
                new SimpleButton(mechanismsJoystick, INTAKE_OPEN_BUTTON),
                new InstantCommand(() -> intake.set(DoubleSolenoid.Value.kReverse)),
                CommandButton.Action.WHEN_PRESSED));

    var defaultCommands = List.<DefaultCommand>of();
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

    return new RobotMap(subsystems, pdp, updater, allCommands, joysticks, USE_CAMERA_SERVER);
  }

  private IntakeTest() {}
}
