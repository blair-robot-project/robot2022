package frc.team449.robot2022.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.multiSubsystem.BooleanSupplierUpdatable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

import java.util.function.BooleanSupplier;

public class PivotingTelescopingClimber extends SubsystemBase implements Loggable {
  /** Distance the climber can travel, in meters */
  public final double distanceTopBottom;

  public final double midDistance;

  private final @NotNull ClimberArm rightArm;
  private final @NotNull ClimberArm leftArm;
  private final @NotNull DoubleSolenoid pivotPiston;
  private final @NotNull BooleanSupplierUpdatable hallSensor;
  private @NotNull ClimberState state;
  /** The current goal */
  private double goal;

  /**
   * @param leftArm Climber's left arm
   * @param rightArm Climber's right arm
   * @param pivotPiston Piston used to pivot climber arms
   * @param hallSensor The hall sensor that detects whether or not one climber arm's reached the
   *     bottom
   * @param distanceTopBottom How much the climber arms extend up
   */
  public PivotingTelescopingClimber(
      @NotNull ClimberArm leftArm,
      @NotNull ClimberArm rightArm,
      @NotNull DoubleSolenoid pivotPiston,
      @NotNull BooleanSupplier hallSensor,
      double distanceTopBottom,
      double midDistance) {
    this.leftArm = leftArm;
    this.rightArm = rightArm;
    this.pivotPiston = pivotPiston;
    this.distanceTopBottom = distanceTopBottom;
    this.midDistance = midDistance;
    this.hallSensor = new BooleanSupplierUpdatable(hallSensor, null);
    // Start arm retracted
    this.state = ClimberState.RETRACTED;
    this.goal = 0;
  }

  @Log.ToString
  @NotNull
  public ClimberState getState() {
    return state;
  }

  public void setState(@NotNull ClimberState state) {
    this.state = state;
  }

  @Log
  public double getGoal() {
    return goal;
  }

  public void setGoal(double goal) {
    if (goal < 0) {
      goal = 0;
      Shuffleboard.addEventMarker(
          "Tried setting climber goal below 0, clipping", EventImportance.kLow);
    } else if (goal > this.distanceTopBottom) {
      goal = this.distanceTopBottom;
      Shuffleboard.addEventMarker(
          "Tried setting climber goal above " + distanceTopBottom + ", clipping",
          EventImportance.kLow);
    }
    this.goal = goal;
    this.leftArm.setGoal(goal);
    this.rightArm.setGoal(goal);
  }

  public void hold() {
    leftArm.getController().reset(leftArm.getMeasurement());
    rightArm.getController().reset(rightArm.getMeasurement());
    leftArm.setGoal(leftArm.getMeasurement());
    rightArm.setGoal(rightArm.getMeasurement());
    this.enable();
  }

  public void pivotTelescopingArmOut() {
    pivotPiston.set(DoubleSolenoid.Value.kReverse);
  }

  public void pivotTelescopingArmIn() {
    pivotPiston.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Directly set velocity. This method must be called every loop if you're using climber. However,
   * sets velocity to 0 if:
   *
   * <ul>
   *   <li>Hall sensor is active, arms are below half the mid climb height limit, and velocity is
   *       negative (going down too far)
   *   <li>Hall sensor is active, climber is stowed, arms are above half the mid climb height limit,
   *       and velocity is positive (going up too far during mid climb)
   * </ul>
   */
  public void set(double velocity) {
    if (velocity <= 0 && this.reachedBottom()) {
      // Don't move further down when already at the bottom
      velocity = 0;
    } else if (this.isStowed() && this.reachedMid()) {
      // Don't move further up when already at the mid climb height limit
      velocity = 0;
    }

    leftArm.set(velocity);
    rightArm.set(velocity);
  }

  /** Whether the arms are vertical */
  public boolean isStowed() {
    return pivotPiston.get() == DoubleSolenoid.Value.kForward;
  }

  public void enable() {
    this.leftArm.enable();
    this.rightArm.enable();
  }

  public void disable() {
    this.leftArm.disable();
    this.rightArm.disable();
  }

  public void stop() {
    this.leftArm.stop();
    this.rightArm.stop();
  }

  /** Clear the previous goal and go to this one */
  public void reset(double goal) {
    this.leftArm.getController().reset(this.leftArm.getMeasurement());
    this.rightArm.getController().reset(this.rightArm.getMeasurement());
    this.leftArm.setGoal(goal);
    this.rightArm.setGoal(goal);

    this.goal = goal;
    System.out.println("resetting to goal " + goal);
  }

  public boolean atGoal() {
    return this.leftArm.getController().atGoal() && this.rightArm.getController().atGoal();
  }

  /**
   * Whether the arms have hit the bottom. The hall effect sensor should be detecting something and
   * the arms must be below half the climber distance. This actually just checks the left arm
   * instead of both arms, though.
   */
  public boolean reachedBottom() {
    return hallSensor.getAsBoolean() && leftArm.getMeasurement() < midDistance / 2;
  }

  /**
   * Whether the arms have reached the mid climb height limit. The hall effect sensor should be
   * detecting something and the arms must be above half the climber distance. This actually just
   * checks the left arm instead of both arms, though.
   */
  public boolean reachedMid() {
    return hallSensor.getAsBoolean() && leftArm.getMeasurement() > midDistance / 2;
  }

  public enum ClimberState {
    EXTENDED,
    RETRACTED,
    MIDDLE
  }
}
