package frc.team449.oi.fieldoriented;

import frc.team449.updatable.Updatable;
import frc.team449.updatable.Updater;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.Nullable;

/** An OI that gives an absolute heading, relative to the field, and a velocity. */
public abstract class OIFieldOriented implements Updatable, Loggable {

  /** The cached linear velocity. */
  private double cachedVel;

  /** The cached angular setpoint. */
  @Nullable private Double cachedTheta;

  protected OIFieldOriented() {
    Updater.subscribe(this);
  }

  /**
   * Get the absolute angle for the robot to move towards.
   *
   * @return An angular setpoint for the robot in degrees, where 0 is pointing at the other
   *     alliance's driver station and 90 is pointing at the left wall when looking out from the
   *     driver station. Returns null if vel is 0.
   */
  @Nullable
  public abstract Double getTheta();

  /**
   * Get the velocity for the robot to go at.
   *
   * @return A velocity from [-1, 1].
   */
  public abstract double getVel();

  /**
   * Get the cached absolute angle for the robot to move towards.
   *
   * @return An angular setpoint for the robot in degrees, where 0 is pointing at the other
   *     alliance's driver station and 90 is pointing at the left wall when looking out from the
   *     driver station. Returns null if vel is 0.
   */
  @Nullable
  @Log
  public Double getThetaCached() {
    return cachedTheta;
  }

  /**
   * Get the cached velocity for the robot to go at.
   *
   * @return A velocity from [-1, 1].
   */
  @Log
  public double getVelCached() {
    return cachedVel;
  }

  /** Updates all cached values with current ones. */
  @Override
  public void update() {
    cachedVel = getVel();
    cachedTheta = getTheta();
  }
}
