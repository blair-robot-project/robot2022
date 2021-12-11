package frc.team449._2021BunnyBot.elevator;

import com.fasterxml.jackson.annotation.JsonCreator;
import edu.wpi.first.wpilibj.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.generalInterfaces.SmartMotor;
import org.jetbrains.annotations.NotNull;

public class OneMotorPulleyElevator extends SubsystemBase {

  @NotNull private final SmartMotor pulleyMotor;
  @NotNull private ElevatorPosition position;
  @NotNull private final ElevatorFeedforward feedforward;
  @NotNull private final ProfiledPIDController pidController;

  /** @param pulleyMotor single motor used for the pulley */
  @JsonCreator
  public OneMotorPulleyElevator(
      @NotNull SmartMotor pulleyMotor,
      @NotNull ElevatorPosition position,
      @NotNull ElevatorFeedforward feedforward,
      @NotNull ProfiledPIDController pidController) {
    this.pulleyMotor = pulleyMotor;
    this.position = position;
    this.feedforward = feedforward;
    this.pidController = pidController;
    this.pulleyMotor.resetPosition();
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

  /**
   * @return the {@link ProfiledPIDController} object or the pid controller used for this elevator
   */
  public ProfiledPIDController getController() {
    return pidController;
  }

  /** @return the position reading on the encoder */
  public double getRawPosition() {
    return pulleyMotor.getPositionUnits();
  }

  /**
   * @param pos the desired position to set the elevator no motion profiling involved, works with
   *     just PID control
   */
  public void moveToPosition(@NotNull ElevatorPosition pos) {
    var calculated = pidController.calculate(this.getRawPosition(), pos.distanceFromBottom);
    System.out.println(calculated);
    pulleyMotor.setVelocity(calculated);
    System.out.println(pulleyMotor.getPositionUnits());
    pulleyMotor.setPositionSetpoint(pos.distanceFromBottom);
    position = pos; // update position
  }

  /**
   * Sets the velocity of the elevator.
   *
   * <p>This allows for fine adjustment via the joystick if the setpoints aren't enough.
   *
   * @param newVelocity the requested new velocity to be set (in m/s)
   */
  public void setVelocityUPS(double newVelocity) {
    pulleyMotor.setVelocityUPS(feedforward.calculate(newVelocity));
  }

  public enum ElevatorPosition {
    // preset positions (RPS)
    // Each crate is 11 inches high (0.2794 meters)
    TOP(0.8382),
    UPPER(0.5588),
    LOWER(0.2794),
    BOTTOM(0.0);

    /** The distance of this position from the bottom in meters */
    public final double distanceFromBottom;

    ElevatorPosition(double distanceFromBottom) {
      this.distanceFromBottom = distanceFromBottom;
    }
  }
}
