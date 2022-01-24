package frc.team449._2022robot.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.jacksonWrappers.WrappedMotor;
import frc.team449.multiSubsystem.SolenoidSimple;
import org.jetbrains.annotations.NotNull;

public class ClimberActuated extends ProfiledPIDSubsystem {
    private final WrappedMotor telescopingArmWinch;
    @NotNull private final double maxDistanceTelescope;
    private final SolenoidSimple pivotingTelescopingArm;
    private ElevatorFeedforward feedforward;
    public ClimberActuated (@NotNull WrappedMotor telescopingArmWinch,
                            @NotNull SolenoidSimple pivotingTelescopingArm,
                            double maxDistanceTelescope,
                            @NotNull ElevatorFeedforward feedforward,
                            @NotNull double kP,
                            @NotNull double kI,
                            @NotNull double kD,
                            @NotNull double telescopingArmMaxVelocity,
                            @NotNull double telescopingArmMaxAcceleration){
        super(new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(telescopingArmMaxVelocity, telescopingArmMaxAcceleration)));
        this.telescopingArmWinch = telescopingArmWinch;
        this.pivotingTelescopingArm = pivotingTelescopingArm;
        this.maxDistanceTelescope = maxDistanceTelescope;
        this.feedforward = feedforward;
    }
    public void extendTelescopingArm(){

    }
    public void retractTelescopingArm(){

    }
    public void pivotTelescopingArmOut(){

    }
    public void pivotTelescopingArmIn(){

    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedForward = feedforward.calculate(setpoint.position, setpoint.velocity);
        telescopingArmWinch.setVoltage(output + feedForward);
    }

    @Override
    protected double getMeasurement() {
        return telescopingArmWinch.encoder.getPosition();
    }
}
