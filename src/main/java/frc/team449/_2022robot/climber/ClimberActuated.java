package frc.team449._2022robot.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.team449.jacksonWrappers.WrappedMotor;
import frc.team449.multiSubsystem.SolenoidSimple;
import org.jetbrains.annotations.NotNull;

public class ClimberActuated extends ProfiledPIDSubsystem {
  private final WrappedMotor telescopingArmWinch;
  private final double maxDistanceTelescope;
  private final double telescopingArmMaxVel;
  private final SolenoidSimple pivotingTelescopingArm;
  private final ElevatorFeedforward feedforward;
  private final DigitalInput topLimitSwitch;
  private final DigitalInput bottomLimitSwitch;

  public ClimberActuated(
      @NotNull WrappedMotor telescopingArmWinch,
      @NotNull SolenoidSimple pivotingTelescopingArm,
      @NotNull DigitalInput topLimitSwitch,
      @NotNull DigitalInput bottomLimitSwitch,
      double maxDistanceTelescope,
      @NotNull ElevatorFeedforward feedforward,
      double kP,
      double kI,
      double kD,
      double telescopingArmMaxVelocity,
      double telescopingArmMaxAcceleration) {
    super(
        new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                telescopingArmMaxVelocity, telescopingArmMaxAcceleration)));
    this.telescopingArmWinch = telescopingArmWinch;
    this.pivotingTelescopingArm = pivotingTelescopingArm;
    this.maxDistanceTelescope = maxDistanceTelescope;
    this.feedforward = feedforward;
    this.topLimitSwitch = topLimitSwitch;
    this.bottomLimitSwitch = bottomLimitSwitch;
    this.telescopingArmMaxVel = telescopingArmMaxVelocity;
  }

  public void extendTelescopingArm() {
    // todo use a lower velocity than the max velocity
    telescopingArmWinch.set(telescopingArmMaxVel);
    while (!topLimitSwitch.get()) {}
    telescopingArmWinch.set(0);
  }

  public void retractTelescopingArm() {
    // todo use a lower velocity than the max velocity
    telescopingArmWinch.set(-telescopingArmMaxVel);
    while (!bottomLimitSwitch.get()) {}
    telescopingArmWinch.set(0);
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

  protected double getMeasurement() {
    return telescopingArmWinch.encoder.getPosition();
  }
}
