package frc.team449.other;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

import java.util.LinkedList;

/** Debouncer that takes the average of the samples in a limited-length buffer. */
public class DebouncerEx implements Loggable {
  private final double bufferSize;
  private final boolean stateWhenTied;
  @Log.ToString @NotNull private final LinkedList<@NotNull Boolean> buffer = new LinkedList<>();
  @Log private int bufferSum;

  /**
   * Constructor for a time given in seconds.
   *
   * @param bufferSize the number of samples to keep in the buffer
   * @param stateWhenTied the state to return when there are an equal number of true and false
   *     samples in the buffer
   */
  public DebouncerEx(final int bufferSize, final boolean stateWhenTied) {
    this.bufferSize = bufferSize;
    this.stateWhenTied = stateWhenTied;
  }

  public DebouncerEx(final int bufferSize) {
    this(bufferSize, false);
  }

  public void update(final boolean currentState) {
    this.buffer.addFirst(currentState);
    this.bufferSum += currentState ? 1 : -1;

    if (this.buffer.size() > this.bufferSize) {
      this.bufferSum -= buffer.removeLast() ? 1 : -1;
    }
  }

  @Log
  public boolean get() {
    if (this.bufferSum == 0) return this.stateWhenTied;
    return this.bufferSum > 0;
  }
}
