package frc.team449.other;

import edu.wpi.first.wpilibj.Timer;
import org.jetbrains.annotations.Contract;

/**
 * A wrapper on {@link System}.currentTimeMillis that caches the time, to avoid calling the
 * currentTimeMillis method.
 */
public class Clock {

  /** The starting time for this clock. */
  private static double startTime;

  /** The time since startTime, in milliseconds. */
  private static double currentTime;

  /** Make constructor private so it can't be called */
  private Clock() {}

  /** Updates the current time. */
  public static synchronized void updateTime() {
    currentTime = Timer.getFPGATimestamp() - startTime;
  }

  /** Sets the start time to the current time. */
  public static synchronized void setStartTime() {
    startTime = Clock.currentTimeSeconds();
  }

  /** The time since the start time, in milliseconds. */
  @Contract(pure = true)
  public static synchronized long currentTimeMillis() {
    return (long)(currentTime * 1000);
  }

  /** The time since the start time, in seconds. */
  @Contract(pure = true)
  public static synchronized double currentTimeSeconds() {
    return currentTime;
  }
}
