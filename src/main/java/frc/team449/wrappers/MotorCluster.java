package frc.team449.wrappers;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import org.jetbrains.annotations.NotNull;

import java.util.List;

/**
 * A cluster of simple motors that act as a single simple motor. Don't use this for talons, use
 * master-slave instead.
 */
public class MotorCluster implements MotorController {

  /**
   * The motors in this cluster. Contains at least 1 element.
   */
  @NotNull
  private final List<MotorController> motors;

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
  public MotorCluster(@JsonProperty(required = true) @NotNull List<MotorController> motors) {
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
  public void set(double velocity) {
    for (var motor : motors) {
      motor.set(velocity);
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
