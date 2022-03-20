package frc.team449;

import edu.wpi.first.wpilibj2.command.Command;
import io.github.oblarg.oblog.Loggable;

import java.util.List;

/**
 * A container class that holds all the commands on the robot, for cleanliness in the map and so
 * that they all appear under the same tab on the dashboard.
 */
public class CommandContainer implements Loggable {
  private final List<Command> robotStartupCommand;

  private final List<Command> autoStartupCommand;

  private final List<Command> teleopStartupCommand;

  private final List<Command> testStartupCommand;

  public CommandContainer(
      List<Command> robotStartupCommand,
      List<Command> autoStartupCommand,
      List<Command> teleopStartupCommand,
      List<Command> testStartupCommand) {
    this.robotStartupCommand = robotStartupCommand;
    this.autoStartupCommand = autoStartupCommand;
    this.teleopStartupCommand = teleopStartupCommand;
    this.testStartupCommand = testStartupCommand;
  }

  public List<Command> getRobotStartupCommand() {
    return this.robotStartupCommand;
  }

  public List<Command> getAutoStartupCommand() {
    return this.autoStartupCommand;
  }

  public List<Command> getTeleopStartupCommand() {
    return this.teleopStartupCommand;
  }

  public List<Command> getTestStartupCommand() {
    return this.testStartupCommand;
  }

  @Override
  public String configureLogName() {
    return "Commands";
  }
}
