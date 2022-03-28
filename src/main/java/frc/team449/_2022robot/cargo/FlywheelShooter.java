package frc.team449._2022robot.cargo;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.motor.WrappedMotor;
import org.jetbrains.annotations.NotNull;

public class FlywheelShooter extends SubsystemBase {
    public final WrappedMotor flywheelMotor;
    /** The desired speed of the flywheel motor */
    private double setpoint;
    private final BangBangController controller = new BangBangController();
    private final SimpleMotorFeedforward feedforward;
    private final double FFdampener;

    public FlywheelShooter(@NotNull WrappedMotor flywheelMotor, double setpoint, SimpleMotorFeedforward feedforward, double FFdampener) {
        this.flywheelMotor = flywheelMotor;
        this.setpoint = setpoint;
        this.feedforward = feedforward;
        this.FFdampener = FFdampener;
    }

    public void shoot() {
        setpoint = 10; // TODO: ACTUALLY GET A TESTED NUMBER FOR THIS
    }

    public void stop() {
        setpoint = 0;
    }

    @Override
    public void periodic() {
        double voltage_multiplier = RobotController.getBatteryVoltage();
        flywheelMotor.setVoltage(controller.calculate(flywheelMotor.encoder.getVelocityUnits(), setpoint) * voltage_multiplier + FFdampener * feedforward.calculate(setpoint));
    }
}
