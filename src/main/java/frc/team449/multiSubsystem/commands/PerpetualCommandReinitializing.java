package frc.team449.multiSubsystem.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import org.jetbrains.annotations.NotNull;

/**
 * Runs another command in perpetuity, ignoring that command's end conditions and reinitializing it
 * when it finishes.
 *
 * @see PerpetualCommand
 */
public class PerpetualCommandReinitializing extends PerpetualCommand {
  /**
   * Creates a new PerpetualCommand. Will run another command in perpetuity, ignoring that command's
   * end conditions, unless this command itself is interrupted.
   *
   * @param command the command to run perpetually
   */
  public PerpetualCommandReinitializing(@NotNull final Command command) {
    super(command);
  }

  @Override
  public void execute() {
    // TODO: How much less jank is this compared to ConditionalCommandDynamic?
    if (this.m_command.isFinished()) {
      this.m_command.end(false);
      this.m_command.initialize();
    }
    super.execute();
  }
}
