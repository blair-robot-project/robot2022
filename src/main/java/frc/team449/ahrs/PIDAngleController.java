package frc.team449.ahrs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * A wrapper around {@link edu.wpi.first.math.controller.PIDController PIDController} that can be
 * used for turning the robot to an angle. <br>
 * For convenience, you can use a {@link PIDAngleControllerBuilder} to construct controllers.
 */
public class PIDAngleController implements Loggable {
  /** On-board PID controller */
  @NotNull protected final PIDController pidController;

  /** The minimum the robot should be able to output, to overcome friction. */
  private final double minimumOutput;

  private final double maximumOutput;

  /** The range in which output is turned off to prevent "dancing" around the setpoint. */
  private final double deadband;

  /** Whether or not the loop is inverted. */
  private final boolean inverted;

  /**
   * A buffer timer for having the loop be on target before it stops running. Can be null for no
   * buffer.
   */
  @Nullable private final Debouncer onTargetBuffer;

  /**
   * Default constructor.
   *
   * @param absoluteTolerance The maximum number of degrees off from the target at which we can be
   *     considered within tolerance.
   * @param onTargetBuffer A buffer timer for having the loop be on target before it stops running.
   *     Can be null for no buffer.
   * @param minimumOutput The minimum output (absolute value) of the loop. Defaults to zero.
   * @param maximumOutput The maximum output (absolute value) of the loop. Can be null, and if it
   *     is, no maximum output is used.
   * @param loopTimeMillis The time, in milliseconds, between each loop iteration. Defaults to 20
   *     ms.
   * @param deadband The deadband around the setpoint, in degrees, within which no output is given
   *     to the motors. Defaults to zero.
   * @param inverted Whether the loop is inverted. Defaults to false.
   * @param kP Proportional gain. Defaults to zero.
   * @param kI Integral gain. Defaults to zero.
   * @param kD Derivative gain. Defaults to zero.
   */
  public PIDAngleController(
      double absoluteTolerance,
      @Nullable Debouncer onTargetBuffer,
      double minimumOutput,
      @Nullable Double maximumOutput,
      @Nullable Integer loopTimeMillis,
      double deadband,
      boolean inverted,
      double kP,
      double kI,
      double kD) {

    // Set P, I and D. I and D will normally be 0 if you're using cascading control, like you should
    // be.
    this.pidController =
        new PIDController(
            kP, kI, kD, loopTimeMillis != null ? loopTimeMillis / 1000. : 20. / 1000.);

    // It's a circle, so it's continuous
    pidController.enableContinuousInput(-180, 180);

    // Set the absolute tolerance to be considered on target within.
    pidController.setTolerance(absoluteTolerance);

    // This is how long we have to be within the tolerance band. Multiply by loop period for time in
    // ms.
    this.onTargetBuffer = onTargetBuffer;

    // Minimum output, the smallest output it's possible to give. One-tenth of your drive's top
    // speed is about
    // right.
    this.minimumOutput = minimumOutput;
    this.maximumOutput = maximumOutput == null ? Double.POSITIVE_INFINITY : maximumOutput;

    // Set a deadband around the setpoint, in degrees, within which don't move, to avoid "dancing"
    this.deadband = deadband;

    // Set whether or not to invert the loop.
    this.inverted = inverted;
  }

  @Config
  public void setP(double p) {
    pidController.setP(p);
  }

  @Config
  public void setD(double d) {
    pidController.setD(d);
  }

  @Log
  public double getSetpoint() {
    return pidController.getSetpoint();
  }

  /** Set setpoint for PID loop to use */
  public void setSetpoint(final double setpoint) {
    pidController.setSetpoint(setpoint);
  }

  /** Calculate the output needed to reach the setpoint, given the current heading */
  public double getOutput(double heading) {
    return processOutput(pidController.calculate(heading));
  }

  @Log
  public double getError() {
    return pidController.getPositionError();
  }

  /**
   * Set controller output to the minimum if it's too small
   *
   * @param controllerOutput PID loop output
   */
  private double processOutput(double controllerOutput) {
    if (controllerOutput > 0) {
      controllerOutput = MathUtil.clamp(controllerOutput, minimumOutput, maximumOutput);
    } else if (controllerOutput < 0) {
      controllerOutput = MathUtil.clamp(controllerOutput, -maximumOutput, -minimumOutput);
    }

    if (inverted) {
      controllerOutput *= -1;
    }

    return controllerOutput;
  }

  /**
   * Deadband the output of the PID loop.
   *
   * @param output The output from the WPILib angular PID loop.
   * @return That output after being deadbanded with the map-given deadband.
   */
  public double deadbandOutput(double output) {
    return Math.abs(pidController.getPositionError()) > deadband ? output : 0;
  }

  /**
   * Whether or not the loop is on target. Use this instead of {@link PIDController}'s onTarget.
   *
   * @return True if on target, false otherwise.
   */
  @Log
  public boolean onTarget() {
    if (onTargetBuffer == null) {
      return pidController.atSetpoint();
    } else {
      return onTargetBuffer.calculate(pidController.atSetpoint());
    }
  }

  /** Reset the error and integral term of the internal PIDController. */
  public void resetController() {
    this.pidController.reset();
  }
}
