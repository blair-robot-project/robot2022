package frc.team449.updatable;

import org.jetbrains.annotations.NotNull;

import java.util.function.Supplier;

/**
 * An updatable holding a single value. Wraps a {@link Supplier} and updates its cache periodically.
 */
public class Cached<T> implements Supplier<T>, Updatable {
  @NotNull private final Supplier<T> source;
  private T cachedValue;

  /**
   * Default constructor
   *
   * @param source the {@link Supplier} from which to obtain values
   * @param initialValue The initial value
   */
  public Cached(@NotNull Supplier<@NotNull T> source, @NotNull T initialValue) {
    this.source = source;
    this.cachedValue = initialValue;
    Updater.subscribe(this);
  }

  /**
   * Constructs a cached object using the supplier to obtain the initial value of the cache
   *
   * @param source The {@link Supplier} from which to obtain values
   */
  public Cached(@NotNull Supplier<@NotNull T> source) {
    this(source, source.get());
  }

  /**
   * Gets the cached value of the supplier.
   *
   * @return a result
   */
  @NotNull
  @Override
  public T get() {
    return this.cachedValue;
  }

  /** Updates the cached value of the supplier. */
  @Override
  public void update() {
    this.cachedValue = this.source.get();
  }
}
