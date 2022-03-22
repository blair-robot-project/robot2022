package frc.team449;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.motor.MotorContainer;
import frc.team449.wrappers.PDP;
import io.github.oblarg.oblog.Loggable;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Iterator;
import java.util.List;

/** The object representing the entire robot. */
public class RobotMap {
  @SuppressWarnings("FieldCanBeLocal")
  @NotNull
  private final List<? extends Subsystem> subsystems;

  @NotNull private final MotorContainer motors = MotorContainer.getInstance();

  @NotNull private final CommandContainer commands;

  @SuppressWarnings("FieldCanBeLocal")
  @NotNull private final List<? extends Loggable> loggables;

  @SuppressWarnings("FieldCanBeLocal")
  @NotNull private final PDP pdp;

  /** Whether the camera server should be run. */
  private final boolean useCameraServer;

  /**
   * Default constructor.
   *
   * @param subsystems The robot's subsystems.
   * @param pdp The PDP
   * @param commands A container to hold all of the robot's commands.
   * @param loggables Other loggables with their own tab
   * @param useCameraServer Whether the camera server should be run. Defaults to false.
   */
  public RobotMap(
      @NotNull List<? extends Subsystem> subsystems,
      @NotNull PDP pdp,
      @NotNull CommandContainer commands,
      @NotNull List<? extends Loggable> loggables,
      boolean useCameraServer) {
    this.pdp = pdp;
    this.useCameraServer = useCameraServer;
    this.subsystems = subsystems;
    this.commands = commands;
    this.loggables = loggables;
  }

  /** @return The commands to be run when first enabled in autonomous mode. */
  @Nullable
  public Iterator<Command> getAutoStartupCommands() {
    if (this.commands.getAutoStartupCommand() == null) {
      return null;
    }
    return this.commands.getAutoStartupCommand().iterator();
  }

  /** @return The commands to be run when first enabled in teleoperated mode. */
  @Nullable
  public Iterator<Command> getTeleopStartupCommands() {
    if (this.commands.getTeleopStartupCommand() == null) {
      return null;
    }
    return this.commands.getTeleopStartupCommand().iterator();
  }

  @Nullable
  public Iterator<Command> getTestStartupCommands() {
    if (this.commands.getTestStartupCommand() == null) {
      return null;
    }
    return this.commands.getTestStartupCommand().iterator();
  }

  /** @return The commands to be run when first enabled. */
  @Nullable
  public Iterator<Command> getRobotStartupCommands() {
    if (this.commands.getRobotStartupCommand() == null) {
      return null;
    }
    return this.commands.getRobotStartupCommand().iterator();
  }

  /** @return Whether the camera server should be run. */
  public boolean useCameraServer() {
    return this.useCameraServer;
  }
}
