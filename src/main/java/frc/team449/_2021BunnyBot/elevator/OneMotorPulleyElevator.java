package frc.team449._2021BunnyBot.elevator;

import com.fasterxml.jackson.annotation.JsonCreator;
import edu.wpi.first.wpilibj.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.generalInterfaces.SmartMotor;
import frc.team449.jacksonWrappers.MappedSparkMaxBase;
import org.jetbrains.annotations.NotNull;

public class OneMotorPulleyElevator extends SubsystemBase {

  @NotNull private final SmartMotor pulleyMotor;
  @NotNull private final ElevatorPosition position;
  @NotNull private final ElevatorFeedforward feedforward;
  @NotNull private final TrapezoidProfile.Constraints constraints;
  @NotNull private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  @NotNull private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  /** @param pulleyMotor single motor used for the pulley */
  @JsonCreator
  public OneMotorPulleyElevator(
      @NotNull MappedSparkMaxBase pulleyMotor,
      @NotNull ElevatorPosition position,
      @NotNull ElevatorFeedforward feedforward,
      @NotNull TrapezoidProfile.Constraints constraints) {
    this.pulleyMotor = pulleyMotor;
    this.position = position;
    this.feedforward = feedforward;
    this.pulleyMotor.resetPosition();
    this.constraints = constraints;
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

  /** @return the position reading on the encoder */
  public double getRawPosition() {
    return pulleyMotor.getPositionUnits();
  }

  /**
   * Uses motion profiling via {@link OneMotorPulleyElevator#calculateNextPosition(double)
   * calculateNextPosition} to move the elevator
   *
   * @author Katie Del Toro
   * @param pos the desired position to set the elevator to
   * @param kDt the time since the profile started
   */
  public void moveToPosition(@NotNull ElevatorPosition pos, double kDt) {
    var distance =
        Math.max(Math.min(pos.distanceFromBottom, ElevatorPosition.TOP.distanceFromBottom), 0);
    goal = new TrapezoidProfile.State(distance, 0);
    setpoint = calculateNextPosition(kDt);
    pulleyMotor.setPositionSetpoint(setpoint.position);
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
    TOP(0.7000), //(0.8382),
    UPPER(0.5588),
    LOWER(0.2794),
//    SPIKE(0.1500),
    BOTTOM(0.0);

    /** The distance of this position from the bottom in meters */
    public final double distanceFromBottom;

    ElevatorPosition(double distanceFromBottom) {
      this.distanceFromBottom = distanceFromBottom;
    }
  }
  /**
   * Uses motion profiling magic to calculate the next interpolated setpoint between the current
   * position and the goal.
   *
   * @author Katie Del Toro
   * @param kDt the time since the profile was started
   * @return a new {@linkplain TrapezoidProfile.State profile state}
   */
  public TrapezoidProfile.State calculateNextPosition(double kDt) {
    TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, setpoint);
    return profile.calculate(kDt);
  }
}
