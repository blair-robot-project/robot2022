package org.usfirst.frc.team449.robot._2021.feeder;

import com.fasterxml.jackson.annotation.JsonCreator;
import org.jetbrains.annotations.NotNull;
import org.usfirst.frc.team449.robot._2020.feeder.commands.DefaultFeederCommand;

import java.util.function.BooleanSupplier;

/**
 * Supplies whether a BallCountingFeederCommand holds balls
 * Needed b/c the counting system was done inside of a command instead of a subsystem,
 *    preventing the use of a BooleanSupplierSubsystemBased
 * To be clear, this exists b/c 2020 code was a bit of a mess, not b/c this is a good solution to the issue
 * If this issue recurs, this code can be adapted to take any boolean-supplying method from any default command
 */
public class FeederSystemBooleanSupplier implements BooleanSupplier {

  /** The default counting command to be checked */
  @NotNull private final DefaultFeederCommand feederCommand;

  /**
   * @param feederCommand the counting command to check balls in
   */
  @JsonCreator
  public FeederSystemBooleanSupplier(@NotNull DefaultFeederCommand feederCommand) {
    this.feederCommand = feederCommand;
  }

  /**
   * Checks whether the supplier is true or false
   *
   * @return Whatever the supplier says. Defaults to false
   */
  @Override
  public boolean getAsBoolean() {
    return feederCommand.hasGotBall();
  }
}
