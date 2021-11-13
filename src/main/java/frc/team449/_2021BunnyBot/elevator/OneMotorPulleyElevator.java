package frc.team449._2021BunnyBot.elevator;

import com.fasterxml.jackson.annotation.JsonCreator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.jacksonWrappers.MappedSparkMax;
import org.jetbrains.annotations.NotNull;

public class OneMotorPulleyElevator extends SubsystemBase {

  @NotNull private final MappedSparkMax pulleyMotor;
  @NotNull private final double maxVelocity;
  @NotNull private ElevatorPosition position;

  /** @param pulleyMotor single motor used for the pulley */
  @JsonCreator
  public OneMotorPulleyElevator(
      @NotNull MappedSparkMax pulleyMotor,
      @NotNull ElevatorPosition position,
      @NotNull double maxVelocity) {
    this.pulleyMotor = pulleyMotor;
    this.position = position;
    this.maxVelocity = maxVelocity;
  }

  /** @return velocity of the elevator motor */
  public double getVelocity() {
    return pulleyMotor.getVelocity();
  }

  /** @return the current position of the elevator */
  @NotNull
  public ElevatorPosition getPosition() {
    return position;
  }

  /** @param pos the desired position to set the elevator */
  public void moveToPosition(@NotNull ElevatorPosition pos) {
    pulleyMotor.setPositionSetpoint(pos.distanceFromBottom);

    System.out.println("Moving to " + pos + " position.");

    // update position
    position = pos;
  }
  /**
   * Sets the velocity of the elevator.
   *
   * <p>This allows for fine adjustment via the joystick if the setpoints aren't enough.
   *
   * @param newVelocity the requested new velocity to be set (in m/s)
   * @return true if velocity set successfully, false if newVelocity was higher than maxVelocity
   */
  public boolean setVelocity(double newVelocity) {
    if (Math.abs(newVelocity) <= Math.abs(this.maxVelocity)) {
      pulleyMotor.setVelocityUPS(newVelocity);
      return true;
    } else {
      return false;
    }
  }

  public enum ElevatorPosition {
    // preset positions
    TOP(0.3),
    UPPER(0.2),
    LOWER(0.1),
    BOTTOM(0.0);

    /** The distance of this position from the bottom in meters */
    public final double distanceFromBottom;

    ElevatorPosition(double distanceFromBottom) {
      this.distanceFromBottom = distanceFromBottom;
    }
  }
}
