package frc.team449.robot2022.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class Climber2022 extends SubsystemBase implements Loggable {
  /** Distance the climber can travel, in meters */
  public final double distanceTopBottom;

  public final double midDistance;

  final @NotNull ClimberArm rightArm;
  final @NotNull ClimberArm leftArm;
  private final @NotNull DoubleSolenoid pivotPiston;

  private final double extendOutput;
  private final double retractOutputFast;
  private final double retractOutputSlow;

  /**
   * @param leftArm Climber's left arm
   * @param rightArm Climber's right arm
   * @param pivotPiston Piston used to pivot climber arms
   * @param distanceTopBottom How much the climber arms extend up
   */
  public Climber2022(
      @NotNull ClimberArm leftArm,
      @NotNull ClimberArm rightArm,
      @NotNull DoubleSolenoid pivotPiston,
      double distanceTopBottom,
      double midDistance,
      double extendOutput,
      double retractOutputFast,
      double retractOutputSlow) {
    this.leftArm = leftArm;
    this.rightArm = rightArm;
    this.pivotPiston = pivotPiston;
    this.distanceTopBottom = distanceTopBottom;
    this.midDistance = midDistance;
    this.extendOutput = extendOutput;
    this.retractOutputFast = retractOutputFast;
    this.retractOutputSlow = retractOutputSlow;
  }

  public void pivotTelescopingArmOut() {
    pivotPiston.set(DoubleSolenoid.Value.kReverse);
  }

  public void pivotTelescopingArmIn() {
    pivotPiston.set(DoubleSolenoid.Value.kForward);
  }

  public void setRetract() {
    if (leftArm.reachedBottom()) {
      leftArm.set(0);
    } else if (leftArm.getHeight() < ClimberConstants.CLIMBER_DIFFERENTIATION_HEIGHT) {
      leftArm.set(retractOutputSlow);
    } else {
      leftArm.set(retractOutputFast);
    }

    if (rightArm.reachedBottom()) {
      rightArm.set(0);
    } else if (rightArm.getHeight() < ClimberConstants.CLIMBER_DIFFERENTIATION_HEIGHT) {
      rightArm.set(retractOutputSlow);
    } else {
      rightArm.set(retractOutputFast);
    }
  }

  public void setExtend() {
    var topLimit =
        this.isStowed() ? ClimberArm.ClimberState.MID_LIMIT : ClimberArm.ClimberState.TOP;
    // During mid climb, don't move further up when already at the mid or high height limit
    if (!leftArm.belowState(topLimit)) {
      leftArm.set(0);
    } else {
      leftArm.set(extendOutput);
    }
    if (!rightArm.belowState(topLimit)) {
      rightArm.set(0);
    } else {
      rightArm.set(extendOutput);
    }
  }

  public void stop() {
    leftArm.set(0);
    rightArm.set(0);
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
    } else {
      var topLimit =
          this.isStowed() ? ClimberArm.ClimberState.MID_LIMIT : ClimberArm.ClimberState.TOP;
      // During mid climb, don't move further up when already at the mid or high height limit
      if (!leftArm.belowState(topLimit)) {
        leftVel = 0;
      }
      if (!leftArm.belowState(topLimit)) {
        rightVel = 0;
      }
    }

    leftArm.set(leftVel);
    rightArm.set(rightVel);
  }

  /** Set output for both arms without checking sensors */
  public void setRaw(double output) {
    leftArm.set(output);
    rightArm.set(output);
  }

  /** Whether the arms are vertical */
  @Log
  public boolean isStowed() {
    return pivotPiston.get() == DoubleSolenoid.Value.kForward;
  }
}
