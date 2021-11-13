package frc.team449.generalInterfaces.simpleMotor;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
import frc.team449._2020.multiSubsystem.SubsystemAnalogMotor;

/**
 * A motor with velocity/voltage control and the ability to enable and disable.
 */
@JsonTypeInfo(
        use = JsonTypeInfo.Id.CLASS,
        include = JsonTypeInfo.As.WRAPPER_OBJECT,
        property = "@class")
public interface SimpleMotor extends SubsystemAnalogMotor {
    /**
     * Enables the motor, if applicable.
     */
    default void enable() {
    }

    /**
     * Set output to a given input.
     * @param input The input to give to the motor.
     * @deprecated use {@link SimpleMotor#setVelocity(double)} instead
     */
    @Override
    @Deprecated
    default void set(final double input) {
        this.setVelocity(input);
    }

    /**
     * Set the velocity for the motor to go at.
     * @param velocity the desired velocity, on [-1, 1].
     */
    void setVelocity(double velocity);

    /**
     * Disables the motor, if applicable.
     */
    @Override
    default void disable() {
    }

    /**
     * Unused.
     */
    enum Type {
        MPSTalon,
        Victor,
        VictorSPX,
    }
}
