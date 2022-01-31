package frc.team449._2022robot.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.team449.jacksonWrappers.WrappedMotor;
import frc.team449.multiSubsystem.SolenoidSimple;
mport io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class PivotingTelescopingClimber extends ProfiledPIDSubsystem {
  private final WrappedMotor telescopingArmWinch;
  private final double telescopingArmMaxVel;
  private final SolenoidSimple pivotingTelescopingArm;
  private final ElevatorFeedforward feedforward;
  private final DigitalInput topLimitSwitch;
  private final DigitalInput bottomLimitSwitch;
  public final double DISTANCE_TOP_BOTTOM;

  public PivotingTelescopingClimber(
          @NotNull WrappedMotor telescopingArmWinch,
          SolenoidSimple pivotingTelescopingArm,
          DigitalInput topLimitSwitch,
          DigitalInput bottomLimitSwitch,
          @NotNull ElevatorFeedforward feedforward,
          double kP,
          double kI,
          double kD,
          double telescopingArmMaxVelocity,
          double telescopingArmMaxAcceleration,
          double distance_top_bottom) {
    super(
        new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                    telescopingArmMaxVelocity,
                    telescopingArmMaxAcceleration
            ))
    );
    this.telescopingArmWinch = telescopingArmWinch;
    this.pivotingTelescopingArm = pivotingTelescopingArm;
    this.feedforward = feedforward;
    this.topLimitSwitch = topLimitSwitch;
    this.bottomLimitSwitch = bottomLimitSwitch;
    this.telescopingArmMaxVel = telescopingArmMaxVelocity;
    DISTANCE_TOP_BOTTOM = distance_top_bottom;
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
//Assign button
  public void pivotTelescopingArmIn() {
    pivotingTelescopingArm.setSolenoid(DoubleSolenoid.Value.kReverse);
  }

  protected void useOutput(double output, TrapezoidProfile.@NotNull State setpoint) {
    double feedForward = feedforward.calculate(setpoint.position, setpoint.velocity);
    telescopingArmWinch.setVoltage(output + feedForward);
  }

  @Log
  public double getMeasurement() {
    return telescopingArmWinch.getPosition();
  }
}

