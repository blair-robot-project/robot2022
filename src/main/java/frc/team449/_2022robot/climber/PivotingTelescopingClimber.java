package frc.team449._2022robot.climber;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class PivotingTelescopingClimber extends SubsystemBase implements Loggable {
  /** Distance the climber can travel, in meters */
  public final double distanceTopBottom;
  private final ClimberArm rightArm;
  private ClimberState state;
  private final ClimberArm leftArm;
  /** The current setpoint, which constantly moves */
  private double setpoint;

  public PivotingTelescopingClimber(
      @NotNull ClimberArm leftArm, @NotNull ClimberArm rightArm, double distanceTopBottom) {
    this.leftArm = leftArm;
    this.rightArm = rightArm;
    this.distanceTopBottom = distanceTopBottom;
    // Start arm retracted
    this.state = ClimberState.RETRACTED;
    this.setpoint = 0;
  }

  @Log.ToString
  public ClimberState getState() {
    return state;
  }

  public void setState(ClimberState state) {
    this.state = state;
  }

  @Log
  public double getSetpoint() {
    return setpoint;
  }

  public void setSetpoint(double setpoint) {
    if (setpoint < 0) {
      setpoint = 0;
      Shuffleboard.addEventMarker(
          "Tried setting climber setpoint below 0, clipping", EventImportance.kLow);
    } else if (setpoint > this.distanceTopBottom) {
      setpoint = this.distanceTopBottom;
      Shuffleboard.addEventMarker(
          "Tried setting climber setpoint above " + distanceTopBottom + ", clipping",
          EventImportance.kLow);
    }
    this.setpoint = setpoint;
    this.leftArm.setSetpoint(setpoint);
    this.rightArm.setSetpoint(setpoint);
  }

  //  public void pivotTelescopingArmOut() {
  //    pivotingTelescopingArm.setSolenoid(DoubleSolenoid.Value.kForward);
  //  }
  //
  //  public void pivotTelescopingArmIn() {
  //    pivotingTelescopingArm.setSolenoid(DoubleSolenoid.Value.kReverse);
  //  }

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

  public enum ClimberState {
    EXTENDED,
    RETRACTED,
    MIDDLE
  }
}
