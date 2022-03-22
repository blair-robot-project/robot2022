package frc.team449.robot2022.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
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

//    if (velocity <= 0) {
//      // Don't move further down when already at the bottom
//      if (leftArm.reachedBottom()) {
//        leftVel = 0;
//      }
//      if (rightArm.reachedBottom()) {
//        rightVel = 0;
//      }
//    } else if (this.isStowed()) {
//      // During mid climb, don't move further up when already at the mid climb height limit
//      if (leftArm.reachedMidLimit()) {
//        System.out.println("Left arm stowed and reached mid limit");
//        leftVel = 0;
//      }
//      if (rightArm.reachedMidLimit()) {
//        System.out.println("Right arm stowed and reached mid limit");
//        rightVel = 0;
//      }
//    }

    leftArm.set(leftVel);
    rightArm.set(rightVel);
  }

  /** Whether the arms are vertical */
  @Log
  public boolean isStowed() {
    return pivotPiston.get() == DoubleSolenoid.Value.kForward;
  }
}
