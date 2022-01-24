package frc.team449.multiSubsystem.commands;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.team449.multiSubsystem.BooleanSupplierUpdatable;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Whenever it is executed, either continues running a command that it is already running or begins
 * running one of the two given commands based on the current state of the given condition.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public final class ConditionalPerpetualCommand {
  /**
   * Create a conditional command that, whenever it is executed, either continues running a command
   * that it is already running or begins running one of the two given commands based on the current
   * state of the given condition.
   *
   * @param onTrue The command to execute when the condition is true
   * @param onFalse The command to execute when the condition is false
   * @param booleanSupplier Supplies the condition
   */
  public static Command createConditionalPerpetualCommand(
      @Nullable Command onTrue,
      @Nullable Command onFalse,
      @NotNull BooleanSupplier booleanSupplier) {
    //todo test if perpetually() works properly here
    return new ConditionalCommand(
            Objects.requireNonNullElse(onTrue, PlaceholderCommand.getInstance()),
            Objects.requireNonNullElse(onFalse, PlaceholderCommand.getInstance()),
            booleanSupplier)
        .perpetually();
  }

  /**
   * Create a {@link ConditionalPerpetualCommand} that only runs a command when the specified
   * condition changes.
   *
   * <p>The condition is not monitored while a command is being run as a result of a change.
   */
  public static Command createConditionalPerpetualCommandChangeBased(
      @NotNull BooleanSupplierUpdatable booleanSupplier,
      @Nullable Command afterBecomingTrue,
      @Nullable Command afterBecomingFalse) {
    // The command to run when the condition changes.
    var cmd =
        new ConditionalCommand(
            Objects.requireNonNullElse(afterBecomingTrue, PlaceholderCommand.getInstance()),
            Objects.requireNonNullElse(afterBecomingFalse, PlaceholderCommand.getInstance()),
            booleanSupplier);

    // A supplier that tests for whether the condition has changed.
    var supplier =
        new BooleanSupplier() {
          private boolean lastState;

          @Override
          public boolean getAsBoolean() {
            booleanSupplier.update();

            boolean current = booleanSupplier.getAsBoolean();
            boolean stateChanged = current != this.lastState;
            this.lastState = current;
            return stateChanged;
          }
        };

    // Don't do anything when the condition isn't changing.
    return ConditionalPerpetualCommand.createConditionalPerpetualCommand(cmd, null, supplier);
  }

  private ConditionalPerpetualCommand() {}
}
