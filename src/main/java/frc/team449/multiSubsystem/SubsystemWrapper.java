package frc.team449.multiSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jetbrains.annotations.NotNull;

/** A wrapper around a plain object to pretend it's a Subsystem */
public final class SubsystemWrapper<T> extends SubsystemBase {
  @NotNull private final T obj;

  public SubsystemWrapper(@NotNull T obj) {
    this.obj = obj;
  }

  public T getWrapped() {
    return obj;
  }

  @Override
  public int hashCode() {
    return obj.hashCode();
  }

  @Override
  public boolean equals(Object other) {
    return other instanceof SubsystemWrapper
        && ((SubsystemWrapper<?>) other).getWrapped().equals(this.obj);
  }
}
