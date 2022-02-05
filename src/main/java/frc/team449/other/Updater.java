package frc.team449.other;

import frc.team449.generalInterfaces.updatable.Updatable;
import org.jetbrains.annotations.NotNull;

import java.util.HashSet;
import java.util.Set;

/**
 * A class for updating cached variables. Run the {@link Updater#run()} method in {@link
 * frc.team449.Robot#teleopPeriodic()}
 */
public final class Updater {

  /** The objects to update. */
  @NotNull private static final Set<Updatable> updatables = new HashSet<>();

  private Updater() {}

  /** Subscribes the specified updatables to being updated. */
  public static void subscribe(Updatable updatable) {
    updatables.add(updatable);
  }

  /** Update all the updatables. */
  public static void run() {
    for (var updatable : updatables) {
      updatable.update();
    }
  }
}
