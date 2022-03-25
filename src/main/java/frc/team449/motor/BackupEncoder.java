package frc.team449.motor;

import org.jetbrains.annotations.NotNull;

/**
 * A wrapper to use when you have one external encoder that's more accurate but may be unplugged and
 * an integrated encoder that's less accurate but is less likely to be unplugged.
 *
 * <p>If the primary encoder's position or velocity is 0 but the integrated encoder's is above a
 * given threshold, it concludes that the primary encoder is broken and switches to using the
 * fallback/integrated encoder
 */
public class BackupEncoder extends Encoder {

  @NotNull private final Encoder primary;
  @NotNull private final Encoder fallback;
  private final double posThreshold;
  private final double velThreshold;

  /** Whether the primary encoder's stopped working */
  private boolean useFallback = false;

  public BackupEncoder(
      @NotNull Encoder primary,
      @NotNull Encoder fallback,
      double posThreshold,
      double velThreshold) {
    super(primary.configureLogName(), 1, 1, 1, false);

    this.primary = primary;
    this.fallback = fallback;
    this.posThreshold = posThreshold;
    this.velThreshold = velThreshold;
  }

  @Override
  public double getPositionNative() {
    var fallbackPos = fallback.getPositionUnits();
    if (useFallback) {
      return fallbackPos;
    } else {
      var primaryPos = primary.getPositionUnits();
      if (primaryPos == 0 && Math.abs(fallbackPos) > posThreshold) {
        this.useFallback = true;
        return fallbackPos;
      } else {
        return primaryPos;
      }
    }
  }

  @Override
  public double getVelocityNative() {
    var fallbackVel = fallback.getVelocityUnits();
    if (useFallback) {
      return fallbackVel;
    } else {
      var primaryVel = primary.getVelocityUnits();
      if (primaryVel == 0 && Math.abs(fallbackVel) > velThreshold) {
        this.useFallback = true;
        return fallbackVel;
      } else {
        return primaryVel;
      }
    }
  }

  @Override
  public void resetPosition(double pos) {
    currentEncoder().resetPosition(pos);
  }

  @Override
  public double nativeToRPS(double nat) {
    return currentEncoder().nativeToRPS(nat);
  }

  @Override
  public double rpsToNative(double rps) {
    return currentEncoder().rpsToNative(rps);
  }

  private Encoder currentEncoder() {
    if (useFallback) {
      return fallback;
    } else {
      return primary;
    }
  }
}
