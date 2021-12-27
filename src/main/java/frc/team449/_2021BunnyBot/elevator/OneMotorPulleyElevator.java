package frc.team449._2021BunnyBot.elevator;

import com.fasterxml.jackson.annotation.JsonCreator;
import edu.wpi.first.wpilibj.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.jacksonWrappers.WrappedEncoder;
import frc.team449.jacksonWrappers.WrappedMotor;
import org.jetbrains.annotations.NotNull;

public class OneMotorPulleyElevator extends SubsystemBase {
  @NotNull private final WrappedMotor pulleyMotor;
  @NotNull private final WrappedEncoder encoder;
  @NotNull private final ElevatorPosition position;
  @NotNull private final ElevatorFeedforward feedforward;
  @NotNull private final TrapezoidProfile.Constraints constraints;
  @NotNull private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  @NotNull private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  @NotNull private final PIDController pidController;

  /** @param pulleyMotor single motor used for the pulley */
  @JsonCreator
  public OneMotorPulleyElevator(
      @NotNull WrappedMotor pulleyMotor,
      @NotNull ElevatorPosition position,
      @NotNull ElevatorFeedforward feedforward,
      @NotNull TrapezoidProfile.Constraints constraints,
      @NotNull PIDController pidController) {
    this.pulleyMotor = pulleyMotor;
    this.encoder = pulleyMotor.encoder;
    this.position = position;
    this.feedforward = feedforward;
    this.encoder.resetPosition();
    this.constraints = constraints;
    this.pidController = pidController;
  }

  /** @return velocity of the elevator motor */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /** @return the current position of the elevator */
  @NotNull
  public ElevatorPosition getPosition() {
    return position;
  }

  /** @return the position reading on the encoder */
  public double getPositionUnits() {
    return encoder.getPositionUnits();
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

    pidController.setSetpoint(setpoint.position);
    while (!pidController.atSetpoint()) {
      pulleyMotor.setVoltage(feedforward.ks + pidController.calculate(encoder.getPositionUnits()));
    }
  }

  /**
   * Sets the velocity of the elevator.
   *
   * <p>This allows for fine adjustment via the joystick if the setpoints aren't enough.
   *
   * @param newVelocity the requested new velocity to be set (in m/s)
   */
  public void setVelocityUPS(double newVelocity) {
    pulleyMotor.setVoltage(
        pidController.calculate(encoder.getVelocity(), encoder.upsToEncoder(newVelocity))
            + feedforward.calculate(newVelocity));
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

  public enum ElevatorPosition {
    // preset positions (RPS)
    // Each crate is 11 inches high (0.2794 meters)
    TOP(0.7000), // (0.8382),
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
}
