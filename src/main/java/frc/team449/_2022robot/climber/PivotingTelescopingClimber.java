package frc.team449._2022robot.climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

import java.util.function.BooleanSupplier;

public class PivotingTelescopingClimber extends SubsystemBase implements Loggable {
  /** Distance the climber can travel, in meters */
  public final double distanceTopBottom;
  private final @NotNull ClimberArm rightArm;
  private @NotNull ClimberState state;
  private final @NotNull ClimberArm leftArm;
  private final @NotNull DoubleSolenoid pivotPiston;
  private final @NotNull BooleanSupplier hallSensor;
  /** The current goal */
  private double goal;

  /**
   *
   * @param leftArm Climber's left arm
   * @param rightArm Climber's right arm
   * @param pivotPiston Piston used to pivot climber arms
   * @param hallSensor The hall sensor that detects whether or not one climber arm's reached the bottom
   * @param distanceTopBottom How much the climber arms extend up
   */
  public PivotingTelescopingClimber(
      @NotNull ClimberArm leftArm,
      @NotNull ClimberArm rightArm,
      @NotNull DoubleSolenoid pivotPiston,
      @NotNull BooleanSupplier hallSensor,
      double distanceTopBottom) {
    this.leftArm = leftArm;
    this.rightArm = rightArm;
    this.pivotPiston = pivotPiston;
    this.distanceTopBottom = distanceTopBottom;
    this.hallSensor = hallSensor;
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

  public void pivotTelescopingArmOut() {
    pivotPiston.set(DoubleSolenoid.Value.kReverse);
  }

  public void pivotTelescopingArmIn() {
    pivotPiston.set(DoubleSolenoid.Value.kForward);
  }

  /** Only for testing/debugging */
  @Deprecated
  public void set(double velocity) {
    leftArm.set(velocity);
    rightArm.set(velocity);
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

  public boolean onTarget() {
    return this.leftArm.getController().atGoal() && this.rightArm.getController().atGoal();
  }

  /** Whether or not the hall sensor is on, meaning one arm hit the bottom */
  public boolean hitBottom() {
    return hallSensor.getAsBoolean();
  }

  public enum ClimberState {
    EXTENDED,
    RETRACTED,
    MIDDLE
  }
}
