package frc.team449._2021BunnyBot.Elevator;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.generalInterfaces.SmartMotor;
import org.jetbrains.annotations.NotNull;

public class OneMotorPulleyElevator extends SubsystemBase {
    @NotNull private final SmartMotor pulleyMotor;
    private double velocity;

    /**
        @param pulleyMotor single motor used for the pulley
        @param velocity the speed the elevator should rise and lower at, should be in interval [-1, 1]
     */
    @JsonCreator
    public OneMotorPulleyElevator(
            @NotNull @JsonProperty(required = true) SmartMotor pulleyMotor,
            @JsonProperty(required = true) double velocity) {
        this.pulleyMotor = pulleyMotor;
        this.velocity = velocity;
    }

    // spin in the same direction of the velocity
    public void setSameVelocity(){
        pulleyMotor.setVelocity(velocity);
    }

    // spin in opposite direction of the velocity
    public void setOppositeVelocity(){
        pulleyMotor.setVelocity(-velocity);
    }

    // returns the currrent velocity of the pulley motor
    public double getVelocity(){
        return pulleyMotor.getVelocity();
    }

    // set to any velocity [-1, 1]
    public void setVelocity(double vel){
        pulleyMotor.setVelocity(vel);
    }
}
