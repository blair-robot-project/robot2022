package frc.team449.components;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.GenericHID;
import frc.team449.ahrs.AHRS;
import frc.team449.other.Clock;
import org.jetbrains.annotations.NotNull;

import java.util.List;

/**
 * A component to rumble controllers based off the jerk measurements from an AHRS (jerk is the
 * derivative of acceleration).
 */
public class AHRSRumbleComponent implements Runnable {

  /** The NavX to get jerk measurements from. */
  @NotNull private final AHRS ahrs;

  /** The joysticks to rumble. */
  @NotNull private final List<? extends GenericHID> joysticks;

  /** The minimum jerk that will trigger rumbling, in meters/second^3. */
  private final double minJerk;

  /**
   * The jerk, in meters/second^3, that's scaled to maximum rumble. All jerks of greater magnitude
   * are capped at 1.
   */
  private final double maxJerk;

  /** Whether the NavX Y-axis measures forwards-back jerk or left-right jerk. */
  private final boolean yIsFrontBack; // TODO why is this never accessed

  /**
   * Variables for the per-call rumble calculation representing the directional accelerations.
   * Fields to avoid garbage collection.
   */
  private double lastFrontBackAccel, lastLeftRightAccel;

  /** The time at which the acceleration was last measured, in seconds. */
  private double timeLastCalled;

  /**
   * Default constructor.
   *
   * @param ahrs The NavX to get jerk measurements from.
   * @param joysticks The things to rumble.
   * @param minJerk The minimum jerk that will trigger rumbling, in meters/(sec^3).
   * @param maxJerk The jerk, in meters/(sec^3), that's scaled to maximum rumble. All jerks of
   *     greater magnitude are capped at 1.
   * @param yIsFrontBack Whether the NavX Y-axis measures forwards-back jerk or left-right jerk.
   *     Defaults to false.
   */
  public AHRSRumbleComponent(
      @NotNull AHRS ahrs,
      @NotNull List<? extends @NotNull GenericHID> joysticks,
      double minJerk,
      double maxJerk,
      boolean yIsFrontBack) {
    this.ahrs = ahrs;
    this.joysticks = joysticks;
    this.minJerk = minJerk;
    this.maxJerk = maxJerk;
    this.yIsFrontBack = yIsFrontBack;
    this.timeLastCalled = 0;
    this.lastFrontBackAccel = 0;
    this.lastLeftRightAccel = 0;
  }

  /** Read the NavX jerk data and rumble the joysticks based off of it. */
  @Override
  public void run() {
    double frontBackAccel = this.ahrs.getYAccel();
    double leftRightAccel = this.ahrs.getXAccel();

    double deltaFrontBack = frontBackAccel - this.lastFrontBackAccel;
    double deltaLeftRight = leftRightAccel - this.lastLeftRightAccel;

    double currTime = Clock.currentTimeSeconds();
    double dt = currTime - this.timeLastCalled;

    // Left is negative jerk, so we subtract it from left so that when we're going left, left is
    // bigger and vice versa
    double leftJerk = Math.abs((deltaFrontBack - deltaLeftRight) / dt);
    double rightJerk = Math.abs((deltaFrontBack + deltaLeftRight) / dt);

    double leftRumble = leftJerk <= this.minJerk ? 0 : (leftJerk - this.minJerk) / this.maxJerk;
    double rightRumble = rightJerk <= this.minJerk ? 0 : (rightJerk - this.minJerk) / this.maxJerk;

    for (var joystick : this.joysticks) {
      joystick.setRumble(GenericHID.RumbleType.kLeftRumble, leftRumble);
      joystick.setRumble(GenericHID.RumbleType.kRightRumble, rightRumble);
    }

    this.lastLeftRightAccel = leftRightAccel;
    this.lastFrontBackAccel = frontBackAccel;
    this.timeLastCalled = currTime;
  }
}
