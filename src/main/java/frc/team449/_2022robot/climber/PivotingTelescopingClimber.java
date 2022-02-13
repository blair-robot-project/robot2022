package frc.team449._2022robot.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.team449.multiSubsystem.SolenoidSimple;
import frc.team449.wrappers.WrappedMotor;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class PivotingTelescopingClimber extends ProfiledPIDSubsystem implements Loggable {
  public final double distanceTopBottom;
  public final WrappedMotor leftArm;
  public final WrappedMotor rightArm;
  private ClimberState state;

  public PivotingTelescopingClimber(
      @NotNull WrappedMotor rightArm,
      @NotNull WrappedMotor leftArm,
      double kP,
      double kI,
      double kD,
      double telescopingArmMaxVelocity,
      double telescopingArmMaxAcceleration,
      double distanceTopBottom) {
    super(
        new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                telescopingArmMaxVelocity, telescopingArmMaxAcceleration)));
    this.rightArm = rightArm;
    this.leftArm = leftArm;
    this.distanceTopBottom = distanceTopBottom;
    // Start arm retracted
    this.state = ClimberState.RETRACTED;
    enable();
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
    return this.getController().getSetpoint().position;
  }

  @Log
  public double getError() {
    return this.getController().getPositionError();
  }

  @Log
  public double getGoal() {
    return this.getController().getGoal().position;
  }

  @Log
  public boolean atGoal() {
    return this.getController().atGoal();
  }

//  public void pivotTelescopingArmOut() {
//    pivotingTelescopingArm.setSolenoid(DoubleSolenoid.Value.kForward);
//  }
//
//  public void pivotTelescopingArmIn() {
//    pivotingTelescopingArm.setSolenoid(DoubleSolenoid.Value.kReverse);
//  }

  public void set(double velocity) {
    leftArm.set(velocity);
    rightArm.set(velocity);
  }

  public void reset() {
    this.leftArm.disable();
    this.rightArm.disable();
    this.leftArm.encoder.resetPosition();
    this.rightArm.encoder.resetPosition();
    this.disable();
  }

  public void resetController() {
    this.getController().reset(this.getMeasurement());
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.@NotNull State setpoint) {
    leftArm.setVoltage(output);
    rightArm.setVoltage(output);
  }

  @Log
  @Override
  public double getMeasurement() {
    return leftArm.encoder.getPositionUnits();
  }

  @Log
  public double getRightMeasurement() {
    return rightArm.encoder.getPositionUnits();
  }

  public enum ClimberState {
    EXTENDED,
    RETRACTED,
    MIDDLE
  }
}
