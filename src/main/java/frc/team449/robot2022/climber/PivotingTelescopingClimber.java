package frc.team449.robot2022.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class PivotingTelescopingClimber extends SubsystemBase implements Loggable {
  /** Distance the climber can travel, in meters */
  public final double distanceTopBottom;

  public final double midDistance;

  final @NotNull ClimberArm rightArm;
  final @NotNull ClimberArm leftArm;
  private final @NotNull DoubleSolenoid pivotPiston;
  private @NotNull ClimberState state;
  /** The current goal */
  private double goal;

  /**
   * @param leftArm Climber's left arm
   * @param rightArm Climber's right arm
   * @param pivotPiston Piston used to pivot climber arms
   * @param distanceTopBottom How much the climber arms extend up
   */
  public PivotingTelescopingClimber(
      @NotNull ClimberArm leftArm,
      @NotNull ClimberArm rightArm,
      @NotNull DoubleSolenoid pivotPiston,
      double distanceTopBottom,
      double midDistance) {
    this.leftArm = leftArm;
    this.rightArm = rightArm;
    this.pivotPiston = pivotPiston;
    this.distanceTopBottom = distanceTopBottom;
    this.midDistance = midDistance;
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
   * doesn't move an arm if:
   *
   * <ul>
   *   <li>Hall sensor is active, arm is below half the mid climb height limit, and velocity is
   *       negative (going down too far)
   *   <li>Hall sensor is active, climber is stowed, arm is above half the mid climb height limit,
   *       and velocity is positive (going up too far during mid climb)
   * </ul>
   */
  public void set(double velocity) {
    double leftVel = velocity;
    double rightVel = velocity;

    if (velocity <= 0) {
      // Don't move further down when already at the bottom
      if (leftArm.reachedBottom()) {
        leftVel = 0;
      }
      if (rightArm.reachedBottom()) {
        rightVel = 0;
      }
    } else if (this.isStowed()) {
      // During mid climb, don't move further up when already at the mid climb height limit
      if (leftArm.reachedMidLimit()) {
        leftVel = 0;
      }
      if (rightArm.reachedMidLimit()) {
        rightVel = 0;
      }
    }

    leftArm.set(leftVel);
    rightArm.set(rightVel);
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

  public enum ClimberState {
    EXTENDED,
    RETRACTED,
    MIDDLE
  }
}
