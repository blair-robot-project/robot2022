package frc.team449._2022robot.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.generalInterfaces.simpleMotor.SimpleMotor;
import frc.team449.jacksonWrappers.WrappedMotor;
import frc.team449.multiSubsystem.SolenoidSimple;
import org.jetbrains.annotations.NotNull;

public class ClimberActuated extends SubsystemBase {
    private final WrappedMotor telescopingArmWinch;
    @NotNull private final double maxDistanceTelescope;
    private final WrappedMotor otherMotor;
    private final SolenoidSimple pivotingTelescopingArm;
    public ClimberActuated (@NotNull WrappedMotor telescopingArmWinch, @NotNull WrappedMotor otherMotor, @NotNull SolenoidSimple pivotingTelescopingArm, double maxDistanceTelescope){
        this.telescopingArmWinch = telescopingArmWinch;
        this.otherMotor = otherMotor;
        this.pivotingTelescopingArm = pivotingTelescopingArm;
        this.maxDistanceTelescope = maxDistanceTelescope;
    }

    public void extendTelescopingArm(){

    }
    public void retractTelescopingArm(){

    }
    public void pivotTelescopingArmOut(){

    }
    public void pivotTelescopingArmIn(){

    }
}
