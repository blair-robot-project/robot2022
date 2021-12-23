package frc.team449.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.jetbrains.annotations.Nullable;

public interface AutonomousCommand extends Command {

  default void setRunTimeSeconds() {}

  @Nullable
  default Double getRunTimeSeconds() {
    return null;
  }

  default Boolean autoFinishedCondition() {
    return null;
  }

  default Command getAutoCommand() {
    if (getRunTimeSeconds() == null && autoFinishedCondition() == null) {
      // assume isFinished condition is sufficient
      return this;
    }
    if (autoFinishedCondition() == null) {
      // run command until wait command is up
      return new WaitCommand(getRunTimeSeconds()).deadlineWith(this);
    }
    if (getRunTimeSeconds() == null) {
      // run command until autofinished is met
      return new WaitUntilCommand(() -> Boolean.TRUE.equals(this.autoFinishedCondition())).deadlineWith(this);
    }
    // run command until either the auto condition is met or the runtime
    return this.raceWith(
        new WaitUntilCommand(() -> Boolean.TRUE.equals(this.autoFinishedCondition())), new WaitCommand(getRunTimeSeconds()));
  }
}
