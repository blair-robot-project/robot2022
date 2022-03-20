package frc.team449.drive.unidirectional.commands;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.oi.fieldoriented.OIFieldOriented;
import org.jetbrains.annotations.NotNull;

import java.util.List;

/** Unidirectional drive with field-oriented control */
public class FieldOrientedUnidirectionalDriveCommand extends CommandBase {

  /** The drive this command is controlling. */
  @NotNull private final DriveUnidirectionalWithGyro subsystem;

  /** The OI giving the input stick values. */
  @NotNull private final OIFieldOriented oi;

  /** The points to snap the PID controller input to. */
  @NotNull private final List<AngularSnapPoint> snapPoints;

  @NotNull private final PIDAngleController controller;

  /**
   * Default constructor
   *
   * @param subsystem The drive to execute this command on.
   * @param oi The OI controlling the robot.
   * @param snapPoints The points to snap the PID controller input to.
   */
  public FieldOrientedUnidirectionalDriveCommand(
      @NotNull DriveUnidirectionalWithGyro subsystem,
      @NotNull OIFieldOriented oi,
      @NotNull List<AngularSnapPoint> snapPoints,
      @NotNull PIDAngleController controller) {
    this.oi = oi;
    this.subsystem = subsystem;
    this.snapPoints = snapPoints;
    this.controller = controller;

    // Needs a requires because it's a default command.
    this.addRequirements(this.subsystem);

    // Logging, but in Spanish.
    Shuffleboard.addEventMarker(
        "Drive Robot bueno", this.getClass().getSimpleName(), EventImportance.kNormal);
    // Logger.addEvent("Drive Robot bueno", this.getClass());
  }

  /** Initialize PIDController and variables. */
  @Override
  public void initialize() {
    // Reset all values of the PIDController and enable it.
    controller.resetController();
    Shuffleboard.addEventMarker(
        "FieldOrientedUnidirectionalDriveCommand init.",
        this.getClass().getSimpleName(),
        EventImportance.kNormal);
    // Logger.addEvent("FieldOrientedUnidirectionalDriveCommand init.", this.getClass());
  }

  /** Set PID setpoint to processed controller setpoint. */
  @Override
  public void execute() {
    Double theta = this.oi.getThetaCached();

    if (theta != null) {
      for (final AngularSnapPoint snapPoint : this.snapPoints) {
        // See if we should snap
        if (snapPoint.getLowerBound() < theta && theta < snapPoint.getUpperBound()) {
          theta = snapPoint.getSnapTo();
          // Break to shorten runtime, we'll never snap twice.
          break;
        }
      }
      controller.setSetpoint(theta);
    }

    // Process or zero the input depending on whether the NavX is being overriden.
    double output =
        this.subsystem.getOverrideGyro() ? 0 : controller.getOutput(subsystem.getAHRS().getCachedHeading());

    // Adjust the heading according to the PID output, it'll be positive if we want to go right.
    this.subsystem.setOutput(this.oi.getVelCached() - output, this.oi.getVelCached() + output);
  }

  /**
   * Run constantly because this is a defaultDrive
   *
   * @return false
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Log when this command ends */
  @Override
  public void end(final boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "FieldOrientedUnidirectionalDriveCommand Interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "FieldOrientedUnidirectionalDriveCommand End.",
        this.getClass().getSimpleName(),
        EventImportance.kNormal);
    // Logger.addEvent("FieldOrientedUnidirectionalDriveCommand End.", this.getClass());
  }

  /** A data-holding class representing an angular setpoint to "snap" the controller output to. */
  protected static class AngularSnapPoint {

    /** The angle to snap the setpoint to, in degrees. */
    private final double snapTo;

    /**
     * The upper bound, below which all angles above snapTo are changed to snapTo. Measured in
     * degrees.
     */
    private final double upperBound;

    /**
     * The lower bound, above which all angles below snapTo are changed to snapTo. Measured in
     * degrees.
     */
    private final double lowerBound;

    /**
     * Default constructor.
     *
     * @param snapTo The angle to snap the setpoint to, in degrees.
     * @param upperBound The upper bound, below which all angles above snapTo are changed to snapTo.
     *     Measured in degrees.
     * @param lowerBound The lower bound, above which all angles below snapTo are changed to snapTo.
     *     Measured in degrees.
     */
    public AngularSnapPoint(final double snapTo, final double upperBound, final double lowerBound) {
      this.snapTo = snapTo;
      this.upperBound = upperBound;
      this.lowerBound = lowerBound;
    }

    /** @return The angle to snap the setpoint to, in degrees. */
    public double getSnapTo() {
      return this.snapTo;
    }

    /**
     * @return The upper bound, below which all angles above snapTo are changed to snapTo. Measured
     *     in degrees.
     */
    public double getUpperBound() {
      return this.upperBound;
    }

    /**
     * @return The lower bound, above which all angles below snapTo are changed to snapTo. Measured
     *     in degrees.
     */
    public double getLowerBound() {
      return this.lowerBound;
    }
  }
}
