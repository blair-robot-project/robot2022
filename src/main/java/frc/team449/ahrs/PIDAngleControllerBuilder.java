package frc.team449.ahrs;

import frc.team449.other.Debouncer;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/** Builder for {@link frc.team449.ahrs.PIDAngleController PIDAngleController} */
public final class PIDAngleControllerBuilder {
  private Double absoluteTolerance;
  private @Nullable Debouncer onTargetBuffer;
  private double minimumOutput = 0;
  private @Nullable Double maximumOutput;
  private @Nullable Integer loopTimeMillis;
  private double deadband = 0;
  private boolean inverted = false;
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  /**
   * The maximum number of degrees off from the target at which we can be considered within
   * tolerance.
   */
  public PIDAngleControllerBuilder absoluteTolerance(double absoluteTolerance) {
    this.absoluteTolerance = absoluteTolerance;
    return this;
  }

  /**
   * A buffer timer for having the loop be on target before it stops running. Can be null for no
   * buffer.
   */
  public PIDAngleControllerBuilder onTargetBuffer(Debouncer onTargetBuffer) {
    this.onTargetBuffer = onTargetBuffer;
    return this;
  }

  /** The minimum output of the loop. Defaults to zero. */
  public PIDAngleControllerBuilder minimumOutput(double minimumOutput) {
    this.minimumOutput = minimumOutput;
    return this;
  }

  /** The maximum output of the loop. Can be null, and if it is, no maximum output is used. */
  public PIDAngleControllerBuilder maximumOutput(Double maximumOutput) {
    this.maximumOutput = maximumOutput;
    return this;
  }

  /** The time, in milliseconds, between each loop iteration. Defaults to 20 ms */
  public PIDAngleControllerBuilder loopTimeMillis(Integer loopTimeMillis) {
    this.loopTimeMillis = loopTimeMillis;
    return this;
  }

  /**
   * The deadband around the setpoint, in degrees, within which no output is given to the motors.
   * Defaults to zero.
   */
  public PIDAngleControllerBuilder deadband(double deadband) {
    this.deadband = deadband;
    return this;
  }

  /** Whether the loop is inverted. Defaults to false. */
  public PIDAngleControllerBuilder inverted(boolean inverted) {
    this.inverted = inverted;
    return this;
  }

  /** Gains for the underlying {@link edu.wpi.first.math.controller.PIDController PIDController} */
  public PIDAngleControllerBuilder pid(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    return this;
  }

  @Contract(value = "-> new", pure = true)
  public PIDAngleControllerBuilder copy() {
    var copy =
        new PIDAngleControllerBuilder()
            .onTargetBuffer(onTargetBuffer)
            .minimumOutput(minimumOutput)
            .maximumOutput(maximumOutput)
            .loopTimeMillis(loopTimeMillis)
            .deadband(deadband)
            .inverted(inverted)
            .pid(kP, kI, kD);
    if (absoluteTolerance != null) {
      copy.absoluteTolerance(absoluteTolerance);
    }
    return copy;
  }

  @Contract(" -> new")
  @NotNull
  public PIDAngleController build() {
    assert absoluteTolerance != null
        : "Absolute tolerance required for " + this.getClass().getSimpleName();
    return new PIDAngleController(
        absoluteTolerance,
        onTargetBuffer,
        minimumOutput,
        maximumOutput,
        loopTimeMillis,
        deadband,
        inverted,
        kP,
        kI,
        kD);
  }
}
