package frc.team449.components;

import com.fasterxml.jackson.annotation.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import java.util.ArrayList;
import java.util.List;

@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
@JsonTypeInfo(
    use = JsonTypeInfo.Id.CLASS,
    include = JsonTypeInfo.As.WRAPPER_OBJECT,
    property = "@class")
public class TrajectoryGenerationQuinticComponent implements TrajectoryGenerationComponent {

  final TrajectoryConstraint constraint;
  final TrajectoryConfig configuration;
  final List<Pose2d> waypoints = new ArrayList<>();
  Trajectory trajectory;

  @JsonCreator
  public TrajectoryGenerationQuinticComponent(
      @JsonProperty(required = true) final DriveUnidirectionalWithGyro drivetrain,
      @JsonProperty(required = true) final double maxSpeedMeters,
      @JsonProperty(required = true) final double maxAccelMeters,
      @JsonProperty(required = true) final List<Pose2d> waypoints,
      boolean reversed) {
    this.constraint =
        new DifferentialDriveVoltageConstraint(
            drivetrain.getFeedforward(), drivetrain.getDriveKinematics(), 12);

    // Create config for trajectory
    this.configuration =
        new TrajectoryConfig(maxSpeedMeters, maxAccelMeters)
            .setKinematics(drivetrain.getDriveKinematics())
            .addConstraint(this.constraint)
            .setReversed(reversed);

    this.waypoints.addAll(waypoints);
  }

  @Override
  public Trajectory getTrajectory() {
    this.trajectory = TrajectoryGenerator.generateTrajectory(this.waypoints, this.configuration);
    return this.trajectory;
  }
}
