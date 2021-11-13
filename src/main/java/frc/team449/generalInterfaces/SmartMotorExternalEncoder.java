package frc.team449.generalInterfaces;

import edu.wpi.first.wpilibj.Encoder;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Represents a {@link SmartMotor} with an external encoder
 */
public interface SmartMotorExternalEncoder extends SmartMotor {

    /**
     * @return Total revolutions for debug purposes
     */
    @Override
    default double encoderPosition() {
        return getEncoder().getDistance();
    }

    Encoder getEncoder();

    /**
     * @return Current RPM for debug purposes
     */
    @Override
    @Log
    default double encoderVelocity() {
        return getEncoder().getRate();
    }

    /**
     * Get the velocity of the motor in MPS.
     * @return The motor's velocity in MPS, or null if no encoder CPR was given.
     */
    @Override
    @Log
    default double getVelocity() {
        return this.encoderToUPS(getEncoder().getRate());
    }

    @Override
    default double getPositionUnits() {
        return encoderToUnit(getEncoder().getDistance());
    }

    @Override
    default void resetPosition() {
        getEncoder().reset();
    }
}
