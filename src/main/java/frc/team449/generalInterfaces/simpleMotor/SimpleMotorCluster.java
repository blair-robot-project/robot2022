package frc.team449.generalInterfaces.simpleMotor;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import org.jetbrains.annotations.NotNull;

import java.util.List;

/**
 * A cluster of simple motors that act as a single simple motor. Don't use this for talons, use
 * master-slave instead.
 */
public class SimpleMotorCluster implements SimpleMotor {

  /**
   * The motors in this cluster. Contains at least 1 element.
   */
  @NotNull
  private final List<SimpleMotor> motors;

  /**
   * Whether or not the cluster is inverted. Individual motors could be inverted on top of that
   */
  private boolean inverted = false;

  /**
   * Default constructor
   *
   * @param motors The motors in this cluster. Must have at least 1 element.
   */
  @JsonCreator
  public SimpleMotorCluster(@JsonProperty(required = true) @NotNull List<SimpleMotor> motors) {
    if (motors.isEmpty()) {
      throw new IllegalArgumentException("motors must have at least 1 element!");
    }
    this.motors = motors;
  }

  /**
   * Set the velocity for the motor to go at.
   *
   * @param velocity the desired velocity, on [-1, 1].
   */
  @Override
  public void setVelocity(double velocity) {
    for (SimpleMotor motor : motors) {
      motor.setVelocity(velocity);
    }
  }

  /**
   * Enables the motor, if applicable.
   */
  @Override
  public void enable() {
    for (SimpleMotor motor : motors) {
      motor.enable();
    }
  }

  @Override
  public double get() {
    return this.motors.get(0).get();
  }

  @Override
  public void setInverted(boolean isInverted) {
    for (var motor : motors) {
      if (this.inverted != isInverted) {
        motor.setInverted(!motor.getInverted());
      }
    }
    this.inverted = isInverted;
  }

  @Override
  public boolean getInverted() {
    return inverted;
  }

  /**
   * Disables the motor, if applicable.
   */
  @Override
  public void disable() {
    for (var motor : motors) {
      motor.disable();
    }
  }

  @Override
  public void stopMotor() {
    for (var motor : motors) {
      motor.stopMotor();
    }
  }
}
