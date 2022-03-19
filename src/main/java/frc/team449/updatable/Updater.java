package frc.team449.updatable;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** A Runnable for updating cached variables. */
public final class Updater {

  /** The objects to update. */
  @NotNull private static final List<Updatable> updatables = new ArrayList<>();

  private Updater() {}

  /** Subscribes the specified updatable to being updated. */
  public static void subscribe(Updatable... updatables) {
    Arrays.stream(updatables)
        .filter(updatable -> !Updater.updatables.contains(updatable))
        .forEach(Updater.updatables::add);
  }

  /** Update all the updatables. */
  public static void run() {
    for (final Updatable updatable : Updater.updatables) {
      updatable.update();
    }
  }
}
