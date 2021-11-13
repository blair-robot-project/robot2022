package frc.team449.drive.unidirectional.commands.AHRS;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team449.components.limelight.LimelightDistanceComponent;
import frc.team449.drive.unidirectional.DriveUnidirectional;
import frc.team449.generalInterfaces.AHRS.SubsystemAHRS;
import frc.team449.generalInterfaces.AHRS.commands.PIDAngleCommand;
import frc.team449.generalInterfaces.limelight.Limelight;
import frc.team449.other.Clock;
import frc.team449.other.Debouncer;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * Turn a certain number of degrees from the current heading, based on input from the limelight
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class NavXTurnToAngleLimelight<T extends Subsystem & DriveUnidirectional & SubsystemAHRS>
        extends NavXTurnToAngleRelative<T> {

    private final Limelight limelight;

    /**
     * Default constructor.
     * @param onTargetBuffer    A buffer timer for having the loop be on target before it stops running.
     *                          Can be null for no buffer.
     * @param absoluteTolerance The maximum number of degrees off from the target at which we can be
     *                          considered within tolerance.
     * @param minimumOutput     The minimum output of the loop. Defaults to zero.
     * @param maximumOutput     The maximum output of the loop. Can be null, and if it is, no maximum
     *                          output is used.
     * @param loopTimeMillis    The time, in milliseconds, between each loop iteration. Defaults to 20
     *                          ms.
     * @param deadband          The deadband around the setpoint, in degrees, within which no output is given
     *                          to the motors. Defaults to zero.
     * @param inverted          Whether the loop is inverted. Defaults to false.
     * @param kP                Proportional gain. Defaults to zero.
     * @param kI                Integral gain. Defaults to zero.
     * @param kD                Derivative gain. Defaults to zero.
     * @param drive             The drive subsystem to execute this command on.
     * @param timeout           How long this command is allowed to run for, in seconds. Needed because
     *                          sometimes floating-point errors prevent termination.
     */
    @JsonCreator
    public NavXTurnToAngleLimelight(
            @JsonProperty(required = true) final double absoluteTolerance,
            @Nullable final Debouncer onTargetBuffer,
            final double minimumOutput,
            @Nullable final Double maximumOutput,
            @Nullable final Integer loopTimeMillis,
            final double deadband,
            final boolean inverted,
            final double offset,
            final double kP,
            final double kI,
            final double kD,
            @NotNull @JsonProperty(required = true) final Limelight limelight,
            @NotNull @JsonProperty(required = true) final T drive,
            @JsonProperty(required = true) final double timeout) {
        super(
                absoluteTolerance,
                onTargetBuffer,
                minimumOutput,
                maximumOutput,
                loopTimeMillis,
                deadband,
                inverted,
                kP,
                kI,
                kD,
                offset, // setpoint
                drive,
                timeout);
        this.limelight = limelight;
    }

    /**
     * Set up the start time and setpoint.
     */
    @Override
    public void initialize() {
        // Setup start time
        this.startTime = Clock.currentTimeMillis();
        Shuffleboard.addEventMarker(
                "NavXTurnToAngleLimelight init.", this.getClass().getSimpleName(), EventImportance.kNormal);
        // Logger.addEvent("NavXRelativeTurnToAngle init.", this.getClass());
        // Do math to setup the setpoint.
        this.setSetpoint(PIDAngleCommand.clipTo180(subsystem.getHeadingCached() - limelight.getX()));
        // System.out.println("Current setpoint = " + limelight.getX());
        final LimelightDistanceComponent distanceComponent =
                new LimelightDistanceComponent(limelight, 20. / 12., 36, 7.5);
        System.out.println(distanceComponent.getAsDouble());
    }

    /**
     * Log when the command ends.
     */
    @Override
    public void end(final boolean interrupted) {
        if (interrupted) {
            Shuffleboard.addEventMarker(
                    "NavXTurnToAngleLimelight interrupted!",
                    this.getClass().getSimpleName(),
                    EventImportance.kNormal);
        }
        // how the heck do we stop this thing help
        Shuffleboard.addEventMarker(
                "NavXRelativeTurnToAngleLimelight end.",
                this.getClass().getSimpleName(),
                EventImportance.kNormal);
    }
}
