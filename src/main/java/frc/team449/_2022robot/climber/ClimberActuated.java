package frc.team449._2022robot.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.generalInterfaces.simpleMotor.SimpleMotor;
import frc.team449.multiSubsystem.SolenoidSimple;
import org.jetbrains.annotations.NotNull;

public class ClimberActuated extends SubsystemBase {
    private final SimpleMotor telescopingArmWinch;
    private final SimpleMotor otherMotor;
    private final SolenoidSimple pivotingTelescopingArm;
    public ClimberActuated (@NotNull SimpleMotor telescopingArmWinch, @NotNull SimpleMotor otherMotor, @NotNull SolenoidSimple pivotingTelescopingArm){
        this.telescopingArmWinch = telescopingArmWinch;
        this.otherMotor = otherMotor;
        this.pivotingTelescopingArm = pivotingTelescopingArm;
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
