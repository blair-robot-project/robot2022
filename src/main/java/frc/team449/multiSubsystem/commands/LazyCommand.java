package frc.team449.multiSubsystem.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.jetbrains.annotations.NotNull;

import java.util.function.Supplier;

/**
 * A Command that instantiates and initializes a wrapped command in its {@link
 * LazyCommand#initialize()} method
 */
public class LazyCommand extends CommandBase {
  private final @NotNull Supplier<Command> cmdCreator;
  /** The actual command, which will be created lazily */
  private Command cmd;

  /**
   * @param cmdCreator A Supplier to create the actual command later
   * @param requirements The subsystems required by the wrapped command. DO NOT FORGET THIS!
   */
  public LazyCommand(@NotNull Supplier<Command> cmdCreator, Subsystem... requirements) {
    addRequirements(requirements);
    this.cmdCreator = cmdCreator;
  }

  @Override
  public void initialize() {
    this.cmd = this.cmdCreator.get();
    this.cmd.initialize();
  }

  @Override
  public void execute() {
    this.cmd.execute();
  }

  @Override
  public void end(boolean interrupted) {
    this.cmd.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return this.cmd.isFinished();
  }
}
