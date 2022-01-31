package frc.team449._2022robot.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.team449.multiSubsystem.SolenoidSimple;
import frc.team449.wrappers.WrappedMotor;
import org.jetbrains.annotations.NotNull;

public class PivotingTelescopingClimber extends ProfiledPIDSubsystem {
  public final double distanceTopBottom;
  private final WrappedMotor telescopingArmWinch;
  private final SolenoidSimple pivotingTelescopingArm;
  private final ElevatorFeedforward feedforward;
  private final DigitalInput topLimitSwitch;
  private final DigitalInput bottomLimitSwitch;
  public boolean extended;

  public PivotingTelescopingClimber(
      @NotNull WrappedMotor telescopingArmWinch,
      SolenoidSimple pivotingTelescopingArm,
      @NotNull DigitalInput topLimitSwitch,
      @NotNull DigitalInput bottomLimitSwitch,
      @NotNull ElevatorFeedforward feedforward,
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
    this.telescopingArmWinch = telescopingArmWinch;
    this.pivotingTelescopingArm = pivotingTelescopingArm;
    this.feedforward = feedforward;
    this.topLimitSwitch = topLimitSwitch;
    this.bottomLimitSwitch = bottomLimitSwitch;
    this.distanceTopBottom = distanceTopBottom;
    extended = false; /** Start arm retracted */
    enable();
  }

  public boolean topLimitSwitchTriggered() {
    return topLimitSwitch.get();
  }

  public boolean bottomLimitSwitchTriggered() {
    return bottomLimitSwitch.get();
  }

  public void pivotTelescopingArmOut() {
    pivotingTelescopingArm.setSolenoid(DoubleSolenoid.Value.kForward);
  }

  public void pivotTelescopingArmIn() {
    pivotingTelescopingArm.setSolenoid(DoubleSolenoid.Value.kReverse);
  }

  protected void useOutput(double output, TrapezoidProfile.@NotNull State setpoint) {
    double feedForward = feedforward.calculate(setpoint.position, setpoint.velocity);
    telescopingArmWinch.setVoltage(output + feedForward);
  }

  public double getMeasurement() {
    return telescopingArmWinch.encoder.getPositionUnits();
  }
}
