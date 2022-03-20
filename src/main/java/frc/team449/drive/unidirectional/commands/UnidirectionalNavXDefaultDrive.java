package frc.team449.drive.unidirectional.commands;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.ahrs.SubsystemAHRS;
import frc.team449.drive.unidirectional.DriveUnidirectional;
import frc.team449.oi.RampComponent;
import frc.team449.oi.unidirectional.OIUnidirectional;
import frc.team449.other.Debouncer;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.function.DoubleUnaryOperator;

/**
 * Drive with arcade drive setup, and when the driver isn't turning, use a NavX to stabilize the
 * robot's alignment.
 */
public class UnidirectionalNavXDefaultDrive<
        T extends Subsystem & DriveUnidirectional & SubsystemAHRS>
    extends CommandBase implements Loggable {
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
  /** Controller to calculate how much to turn */
  @NotNull private final PIDAngleController controller;

  /**
   * Default constructor
   *
   * @param maxAngularVelToEnterLoop The maximum angular velocity, in degrees/sec, at which the loop
   *     will be entered. Defaults to 180.
   * @param driveStraightLoopEntryTimer The buffer timer for starting to drive straight.
   * @param subsystem The drive to execute this command on.
   * @param oi The OI controlling the robot.
   * @param rampComponent The acceleration-limiting ramp for the output to the drive. Defaults to no
   *     ramp.
   * @param controller Controller used to actually turn
   */
  public UnidirectionalNavXDefaultDrive(
      @Nullable Double maxAngularVelToEnterLoop,
      @NotNull Debouncer driveStraightLoopEntryTimer,
      @NotNull T subsystem,
      @NotNull OIUnidirectional oi,
      @Nullable RampComponent rampComponent,
      @NotNull PIDAngleController controller) {
    this.oi = oi;
    this.subsystem = subsystem;
    this.leftRamp = rampComponent;
    this.rightRamp =
        rampComponent != null ? rampComponent.copy() : null; // We want the same settings but
    // different objects, so we clone

    this.driveStraightLoopEntryTimer = driveStraightLoopEntryTimer;
    this.maxAngularVelToEnterLoop =
        maxAngularVelToEnterLoop != null ? maxAngularVelToEnterLoop : 180;

    this.controller = controller;

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
    controller.resetController();
    Shuffleboard.addEventMarker(
        "UnidirectionalNavXArcadeDrive init.",
        this.getClass().getSimpleName(),
        EventImportance.kNormal);

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
            && Math.abs(this.subsystem.getAHRS().getCachedAngularVelocity()) <= this.maxAngularVelToEnterLoop)) {
      // Switch to driving straight
      this.drivingStraight = true;
      // Set the setpoint to the current heading and reset the AHRS
      controller.resetController();
      controller.setSetpoint(this.subsystem.getAHRS().getCachedHeading());
    }

    // Update the controller with the current heading
    // Why exactly this line is necessary is beyond me
    controller.getOutput(subsystem.getAHRS().getCachedHeading());
    // Get the outputs
    double leftOutput = this.oi.getLeftRightOutput()[0];
    double rightOutput = this.oi.getLeftRightOutput()[1];
    // Ramp if it exists
    if (this.leftRamp != null && this.rightRamp != null) {
      leftOutput = this.leftRamp.applyAsDouble(leftOutput);
      rightOutput = this.rightRamp.applyAsDouble(rightOutput);
    }

    // If we're driving straight..
    double processedOutput;
    double finalOutput;
    if (this.drivingStraight) {
      // Process the output (minimumOutput, deadband, etc.)
      processedOutput = controller.getOutput(subsystem.getAHRS().getCachedHeading());

      // Deadband if we're stationary
      if (leftOutput == 0 && rightOutput == 0) {
        finalOutput = controller.deadbandOutput(processedOutput);
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
