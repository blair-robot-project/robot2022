package frc.team449._2021BunnyBot.elevator;

import com.fasterxml.jackson.annotation.JsonCreator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.generalInterfaces.SmartMotor;
import org.jetbrains.annotations.NotNull;

public class OneMotorPulleyElevator extends SubsystemBase {

  @NotNull private final SmartMotor pulleyMotor;
  @NotNull private ElevatorPosition position;

  /** @param pulleyMotor single motor used for the pulley */
  @JsonCreator
  public OneMotorPulleyElevator(
      @NotNull SmartMotor pulleyMotor, @NotNull ElevatorPosition position) {
    this.pulleyMotor = pulleyMotor;
    this.position = position;
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
   */
  public void setVelocityUPS(double newVelocity) {
    pulleyMotor.setVelocityUPS(newVelocity);
  }

  public enum ElevatorPosition {
    // preset positions (RPS)
    TOP(2),
    UPPER(1.25),
    LOWER(0.75),
    BOTTOM(0.0);

    /** The distance of this position from the bottom in meters */
    public final double distanceFromBottom;

    ElevatorPosition(double distanceFromBottom) {
      this.distanceFromBottom = distanceFromBottom;
    }
  }
}
