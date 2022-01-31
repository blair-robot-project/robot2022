package frc.team449.components;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import frc.team449.generalInterfaces.rumbleable.Rumbleable;
import frc.team449.wrappers.AHRS;
import frc.team449.other.Clock;
import java.util.List;
import org.jetbrains.annotations.NotNull;

/** A component to rumble controllers based off the jerk measurements from an AHRS. */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class AHRSRumbleComponent implements Runnable {

  /** The NavX to get jerk measurements from. */
  @NotNull private final AHRS ahrs;

  /** The things to rumble. */
  @NotNull private final List<Rumbleable> rumbleables;

  /** The minimum jerk that will trigger rumbling, in meters/second^3. */
  private final double minJerk;

  /**
   * The jerk, in meters/second^3, that's scaled to maximum rumble. All jerks of greater magnitude
   * are capped at 1.
   */
  private final double maxJerk;

  /** Whether the NavX Y-axis measures forwards-back jerk or left-right jerk. */
  private final boolean yIsFrontBack; // TODO why is this never accessed

  /** Whether to invert the left-right jerk measurement. */
  private final boolean invertLeftRight;

  /**
   * Variables for the per-call rumble calculation representing the directional accelerations.
   * Fields to avoid garbage collection.
   */
  private double lastFrontBackAccel, lastLeftRightAccel;

  /** The time at which the acceleration was last measured. */
  private long timeLastCalled;

  /**
   * Default constructor.
   *
   * @param ahrs The NavX to get jerk measurements from.
   * @param rumbleables The things to rumble.
   * @param minJerk The minimum jerk that will trigger rumbling, in meters/(sec^3).
   * @param maxJerk The jerk, in meters/(sec^3), that's scaled to maximum rumble. All jerks of
   *     greater magnitude are capped at 1.
   * @param yIsFrontBack Whether the NavX Y-axis measures forwards-back jerk or left-right jerk.
   *     Defaults to false.
   * @param invertLeftRight Whether to invert the left-right jerk measurement. Defaults to false.
   */
  @JsonCreator
  public AHRSRumbleComponent(
      @NotNull @JsonProperty(required = true) final AHRS ahrs,
      @NotNull @JsonProperty(required = true) final List<Rumbleable> rumbleables,
      @JsonProperty(required = true) final double minJerk,
      @JsonProperty(required = true) final double maxJerk,
      final boolean yIsFrontBack,
      final boolean invertLeftRight) {
    this.ahrs = ahrs;
    this.rumbleables = rumbleables;
    this.minJerk = minJerk;
    this.maxJerk = maxJerk;
    this.yIsFrontBack = yIsFrontBack;
    this.invertLeftRight = invertLeftRight;
    this.timeLastCalled = 0;
    this.lastFrontBackAccel = 0;
    this.lastLeftRightAccel = 0;
  }

  /** Read the NavX jerk data and rumble the joysticks based off of it. */
  @Override
  public void run() {
    double frontBack;
    double leftRight;
    frontBack = Math.abs(this.ahrs.getYAccel());
    leftRight = this.ahrs.getXAccel() * (this.invertLeftRight ? -1 : 1);

    // Left is negative jerk, so we subtract it from left so that when we're going left, left is
    // bigger and vice
    // versa
    double left =
        ((frontBack - this.lastFrontBackAccel) - (leftRight - this.lastLeftRightAccel))
            / (Clock.currentTimeMillis() - this.timeLastCalled);
    double right =
        ((frontBack - this.lastFrontBackAccel) + (leftRight - this.lastLeftRightAccel))
            / (Clock.currentTimeMillis() - this.timeLastCalled);

    if (left > this.minJerk) {
      left = (left - this.minJerk) / this.maxJerk;
    } else {
      left = 0;
    }

    if (right > this.minJerk) {
      right = (right - this.minJerk) / this.maxJerk;
    } else {
      right = 0;
    }

    for (final Rumbleable rumbleable : this.rumbleables) {
      rumbleable.rumble(left, right);
    }

    this.lastLeftRightAccel = leftRight;
    this.lastFrontBackAccel = frontBack;
    this.timeLastCalled = Clock.currentTimeMillis();
  }
}
