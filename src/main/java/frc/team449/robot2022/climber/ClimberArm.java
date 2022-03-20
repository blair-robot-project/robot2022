package frc.team449.robot2022.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.team449.motor.WrappedMotor;
import frc.team449.multiSubsystem.BooleanSupplierUpdatable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

import java.util.function.BooleanSupplier;

/**
 * A single arm of the 2022 climber. Periodically checks if the hall sensor is active. If it is,
 * then it determines whether the arm is at the bottom or mid climb height limit, and resets its
 * motor's encoder's position based on that
 */
public class ClimberArm extends ProfiledPIDSubsystem implements Loggable {
  private final @NotNull WrappedMotor motor;
  private final @NotNull ElevatorFeedforward feedforward;
  private final @NotNull BooleanSupplierUpdatable hallSensor;
  private final double sensorDifferentiationHeight;
  private final double midClimbLimit;
  private double prevOutput = 0.0;
  @Log.ToString private @NotNull PivotingTelescopingClimber.ClimberState state =
      PivotingTelescopingClimber.ClimberState.BOTTOM;

  /**
   * @param motor Winch motor used for retracting/extending arm
   * @param controller Profiled PID controller used for controlling this arm. Unused currently
   * @param feedforward Feedforward
   * @param sensorDifferentiationHeight If the arm is above this height and the hall sensor is on,
   *     the arm is considered to be at the mid climb height limit. If the arm is below this height
   *     and the hall effect sensor is on, the arm is considered to be at the bottom
   * @param midClimbLimit The height limit for mid climb
   * @param hallSensor Hall effect sensor to determine if the climber is at either the bottom or mid
   *     climb height limit
   */
  public ClimberArm(
      @NotNull WrappedMotor motor,
      @NotNull ProfiledPIDController controller,
      @NotNull ElevatorFeedforward feedforward,
      double sensorDifferentiationHeight,
      double midClimbLimit,
      @NotNull BooleanSupplier hallSensor) {
    super(controller);
    this.motor = motor;
    this.feedforward = feedforward;
    this.sensorDifferentiationHeight = sensorDifferentiationHeight;
    this.midClimbLimit = midClimbLimit;
    this.hallSensor = new BooleanSupplierUpdatable(hallSensor, null);
  }

  /** Whether this arm's reached the bottom. Based on encoder position */
  @Log
  public boolean reachedBottom() {
    return state == PivotingTelescopingClimber.ClimberState.BOTTOM;
  }

  /** Whether this arm's reached the mid climb height limit. Based on encoder position */
  @Log
  public boolean reachedMidLimit() {
    return state == PivotingTelescopingClimber.ClimberState.MID_LIMIT
        || state == PivotingTelescopingClimber.ClimberState.ABOVE_MID;
  }

  @Override
  protected void useOutput(double output, TrapezoidProfile.State setpoint) {
    this.motor.setVoltage(output + feedforward.calculate(setpoint.velocity));
  }

  @Override
  public double getMeasurement() {
    return this.motor.getPositionUnits();
  }

  public void stop() {
    this.getController().reset(getMeasurement());
  }

  @Override
  public void periodic() {
    super.periodic();

    // Reset encoder position based on the Hall effect sensors
    if (this.hallSensor.getAsBoolean()) {
      if (state == PivotingTelescopingClimber.ClimberState.BETWEEN) {
        if (prevOutput > 0) {
          this.state = PivotingTelescopingClimber.ClimberState.MID_LIMIT;
        } else {
          this.state = PivotingTelescopingClimber.ClimberState.BOTTOM;
        }
      } else if (state == PivotingTelescopingClimber.ClimberState.ABOVE_MID) {
        if (prevOutput < 0) {
          this.state = PivotingTelescopingClimber.ClimberState.MID_LIMIT;
        }
      }
    } else {
      if (state == PivotingTelescopingClimber.ClimberState.BOTTOM) {
        if (prevOutput > 0) {
          this.state = PivotingTelescopingClimber.ClimberState.BETWEEN;
        }
      } else if (state == PivotingTelescopingClimber.ClimberState.MID_LIMIT) {
        if (prevOutput < 0) {
          this.state = PivotingTelescopingClimber.ClimberState.BETWEEN;
        } else {
          this.state = PivotingTelescopingClimber.ClimberState.ABOVE_MID;
        }
      }
    }
  }

  public void set(double velocity) {
    this.motor.set(velocity);
    this.prevOutput = velocity;
  }

  @Override
  public String configureLogName() {
    return "ClimberArm" + motor.configureLogName();
  }
}
