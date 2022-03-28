package frc.team449._2022robot.cargo;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jetbrains.annotations.NotNull;

public class FlywheelShooter extends SubsystemBase {
    public final MotorController flywheelMotor;
    /** The desired speed of the flywheel motor */
    private final double setpoint;
    private final BangBangController controller = new BangBangController();
    private final Counter encoder;
    private final SimpleMotorFeedforward feedforward;

    public FlywheelShooter(@NotNull MotorController flywheelMotor, double setpoint, Counter encoder, double kS, double kA, double kV) {
        this.flywheelMotor = flywheelMotor;
        this.setpoint = setpoint;
        this.encoder = encoder;
        this.feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public void shoot() {
        flywheelMotor.setVoltage(controller.calculate(encoder.getRate(), setpoint) * 12.0 + 0.9 * feedforward.calculate(setpoint));
    }

    public void stop() {
        flywheelMotor.set(0);
    }
}
