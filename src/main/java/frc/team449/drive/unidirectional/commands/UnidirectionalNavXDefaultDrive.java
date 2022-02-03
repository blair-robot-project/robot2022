package frc.team449.drive.unidirectional.commands;

import com.fasterxml.jackson.annotation.*;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.drive.unidirectional.DriveUnidirectional;
import frc.team449.generalInterfaces.ahrs.SubsystemAHRS;
import frc.team449.generalInterfaces.ahrs.commands.PIDAngleCommand;
import frc.team449.generalInterfaces.doubleUnaryOperator.RampComponent;
import frc.team449.oi.unidirectional.OIUnidirectional;
import frc.team449.other.Debouncer;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleUnaryOperator;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * Drive with arcade drive setup, and when the driver isn't turning, use a NavX to stabilize the
 * robot's alignment.
 */
@JsonTypeInfo(
    use = JsonTypeInfo.Id.CLASS,
    include = JsonTypeInfo.As.WRAPPER_OBJECT,
    property = "@class")
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class UnidirectionalNavXDefaultDrive<
        T extends Subsystem & DriveUnidirectional & SubsystemAHRS>
    extends PIDAngleCommand implements Loggable {
  /** The drive this command is controlling. */
  @NotNull @Log.Exclude protected final T subsystem;
  /** The OI giving the input stick values. */
  @NotNull protected final OIUnidirectional oi;
  /**
   * The maximum velocity for the robot to be at in order to switch to driveStraight, in degrees/sec
   */
  private final double maxAngularVelToEnterLoop;
  /**
   * A bufferTimer so we only switch to driving straight when the conditions are met for a certain
   * period of time.
   */
  @NotNull private final Debouncer driveStraightLoopEntryTimer;
  /**
   * Acceleration-limiting ramps for the left and right sides of the drive, respectively. Null for
   * no ramp.
   */
  @Nullable private final DoubleUnaryOperator leftRamp, rightRamp;
  /** Whether or not we should be using the NavX to drive straight stably. */
  private boolean drivingStraight;

  /**
   * Default constructor
   *
   * @param onTargetBuffer A buffer timer for having the loop be on target before it stops running.
   *     Can be null for no buffer.
   * @param absoluteTolerance The maximum number of degrees off from the target at which we can be
   *     considered within tolerance.
   * @param minimumOutput The minimum output of the loop. Defaults to zero.
   * @param maximumOutput The maximum output of the loop. Can be null, and if it is, no maximum
   *     output is used.
   * @param loopTimeMillis The time, in milliseconds, between each loop iteration. Defaults to 20
   *     ms.
   * @param deadband The deadband around the setpoint, in degrees, within which no output is given
   *     to the motors. Defaults to zero.
   * @param maxAngularVelToEnterLoop The maximum angular velocity, in degrees/sec, at which the loop
   *     will be entered. Defaults to 180.
   * @param inverted Whether the loop is inverted. Defaults to false.
   * @param kP Proportional gain. Defaults to zero.
   * @param kI Integral gain. Defaults to zero.
   * @param kD Derivative gain. Defaults to zero.
   * @param driveStraightLoopEntryTimer The buffer timer for starting to drive straight.
   * @param subsystem The drive to execute this command on.
   * @param oi The OI controlling the robot.
   * @param rampComponent The acceleration-limiting ramp for the output to the drive. Defaults to no
   *     ramp.
   */
  @JsonCreator
  public UnidirectionalNavXDefaultDrive(
      @JsonProperty(required = true) final double absoluteTolerance,
      @Nullable final Debouncer onTargetBuffer,
      final double minimumOutput,
      @Nullable final Double maximumOutput,
      @Nullable final Integer loopTimeMillis,
      final double deadband,
      @Nullable final Double maxAngularVelToEnterLoop,
      final boolean inverted,
      final double kP,
      final double kI,
      final double kD,
      @NotNull @JsonProperty(required = true) final Debouncer driveStraightLoopEntryTimer,
      @NotNull @JsonProperty(required = true) final T subsystem,
      @NotNull @JsonProperty(required = true) final OIUnidirectional oi,
      @Nullable final RampComponent rampComponent) {
    // Assign stuff
    super(
        absoluteTolerance,
        onTargetBuffer,
        minimumOutput,
        maximumOutput,
        loopTimeMillis,
        deadband,
        inverted,
        subsystem,
        kP,
        kI,
        kD);
    this.oi = oi;
    this.subsystem = subsystem;
    this.leftRamp = rampComponent;
    this.rightRamp =
        rampComponent != null ? rampComponent.copy() : null; // We want the same settings but
    // different objects, so we clone

    this.driveStraightLoopEntryTimer = driveStraightLoopEntryTimer;
    this.maxAngularVelToEnterLoop =
        maxAngularVelToEnterLoop != null ? maxAngularVelToEnterLoop : 180;

    // Needs a requires because it's a default command.
    this.addRequirements(this.subsystem);

    // Logging, but in Spanish.
    Shuffleboard.addEventMarker(
        "Drive Robot bueno", this.getClass().getSimpleName(), EventImportance.kNormal);
  }

  /** Initialize PIDController and variables. */
  @Override
  public void initialize() {
    // Reset all values of the PIDController and enable it.
    this.getController().reset();
    Shuffleboard.addEventMarker(
        "UnidirectionalNavXArcadeDrive init.",
        this.getClass().getSimpleName(),
        EventImportance.kNormal);
    // Logger.addEvent("UnidirectionalNavXArcadeDrive init.", this.getClass());

    // Initial assignment
    this.drivingStraight = false;
  }

  /** Decide whether or not we should be in free drive or straight drive. */
  @Override
  public void execute() {
    // If we're driving straight but the driver tries to turn or overrides the AHRS:
    if (this.drivingStraight
        && (!this.oi.commandingStraight() || this.subsystem.getOverrideGyro())) {
      // Switch to free drive
      this.drivingStraight = false;
    }
    // If we're free driving and the driver stops turning:
    else if (this.driveStraightLoopEntryTimer.get(
        !(this.subsystem.getOverrideGyro())
            && !(this.drivingStraight)
            && this.oi.commandingStraight()
            && Math.abs(this.subsystem.getAngularVelCached()) <= this.maxAngularVelToEnterLoop)) {
      // Switch to driving straight
      this.drivingStraight = true;
      // Set the setpoint to the current heading and reset the AHRS
      this.getController().reset();
      this.setSetpoint(this.subsystem.getHeadingCached());
    }

    // Get the outputs
    double rawOutput = this.getRawOutput();
    double leftOutput = this.oi.getLeftRightOutputCached()[0];
    double rightOutput = this.oi.getLeftRightOutputCached()[1];

    // Ramp if it exists
    if (this.leftRamp != null && this.rightRamp != null) {
      leftOutput = this.leftRamp.applyAsDouble(leftOutput);
      rightOutput = this.rightRamp.applyAsDouble(rightOutput);
    }
    System.out.println(leftOutput + " and " + rightOutput);

    // If we're driving straight..
    double processedOutput;
    double finalOutput;
    if (this.drivingStraight) {
      // Process the output (minimumOutput, deadband, etc.)
      processedOutput = this.getOutput();

      // Deadband if we're stationary
      if (leftOutput == 0 && rightOutput == 0) {
        finalOutput = this.deadbandOutput(processedOutput);
      } else {
        finalOutput = processedOutput;
      }

      // Adjust the heading according to the PID output, it'll be positive if we want to go right.
      this.subsystem.setOutput(leftOutput - finalOutput, rightOutput + finalOutput);
    }
    // If we're free driving...
    else {
      // Set the throttle to normal arcade throttle.
      this.subsystem.setOutput(leftOutput, rightOutput);
    }
  }

  /**
   * Run constantly because this is a defaultDrive
   *
   * @return false
   */
  @Override
  // @Log
  public boolean isFinished() {
    return false;
  }

  /** Log when this command ends */
  @Override
  public void end(final boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "UnidirectionalNavXArcadeDrive Interrupted! Stopping the robot.",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    this.subsystem.fullStop();
    Shuffleboard.addEventMarker(
        "UnidirectionalNavXArcadeDrive End.",
        this.getClass().getSimpleName(),
        EventImportance.kNormal);
    // Logger.addEvent("UnidirectionalNavXArcadeDrive End.", this.getClass());
  }

  // @Log
  public boolean isDrivingStraight() {
    return this.drivingStraight;
  }
}
