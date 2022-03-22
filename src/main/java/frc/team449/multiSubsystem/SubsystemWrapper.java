package frc.team449.multiSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jetbrains.annotations.NotNull;

import java.util.Objects;

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
    return Objects.hash(obj);
  }

  @Override
  public boolean equals(Object other) {
    return other instanceof SubsystemWrapper
        && Objects.equals(this.obj, ((SubsystemWrapper<?>) other).getWrapped());
  }

  @Override
  public String toString() {
    return "SubsystemWrapper(" + this.obj + ")";
  }
}
