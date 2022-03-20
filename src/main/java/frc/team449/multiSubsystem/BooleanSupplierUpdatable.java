package frc.team449.multiSubsystem;

import frc.team449.updatable.Updatable;
import frc.team449.updatable.Updater;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Objects;
import java.util.function.BooleanSupplier;

/** Wraps a {@link BooleanSupplier} and only updates its result when told to. */
public class BooleanSupplierUpdatable implements BooleanSupplier, Updatable {
  @NotNull private final BooleanSupplier source;
  private boolean cachedValue;

  /**
   * Default constructor
   *
   * @param source the {@link BooleanSupplier} from which to obtain values
   */
  public BooleanSupplierUpdatable(
      @NotNull final BooleanSupplier source, @Nullable final Boolean initialValue) {
    this.source = source;
    this.cachedValue = Objects.requireNonNullElseGet(initialValue, source::getAsBoolean);
    Updater.subscribe(this);
  }

  /**
   * Gets the cached value of the supplier.
   *
   * @return a result
   */
  @Override
  public boolean getAsBoolean() {
    return this.cachedValue;
  }

  /** Updates the cached value of the supplier. */
  @Override
  public void update() {
    this.cachedValue = this.source.getAsBoolean();
  }
}
