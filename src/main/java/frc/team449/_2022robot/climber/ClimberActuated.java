package frc.team449._2022robot.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.generalInterfaces.simpleMotor.SimpleMotor;

public class ClimberActuated extends SubsystemBase {
    private final SimpleMotor telescopingArmWinch;
    private final SimpleMotor otherMotor;

    public ClimberActuated (SimpleMotor telescopingArmWinch, SimpleMotor otherMotor){
        this.telescopingArmWinch = telescopingArmWinch;
        this.otherMotor = otherMotor;
    }

    public void midRungClimb(){
        // complete first half of the climb
    }

    public void highRungClimb(){
        // complete second half of the climb
    }
}
