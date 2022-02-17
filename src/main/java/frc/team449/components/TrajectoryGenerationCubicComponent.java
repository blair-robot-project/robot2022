package frc.team449.components;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.List;

@JsonTypeInfo(
    use = JsonTypeInfo.Id.CLASS,
    include = JsonTypeInfo.As.WRAPPER_OBJECT,
    property = "@class")
public class TrajectoryGenerationCubicComponent implements TrajectoryGenerationComponent {

  final DriveUnidirectionalWithGyro drivetrain;
  final double maxSpeedMeters;
  final double maxAccelMeters;
  final Trajectory trajectory;
  double maxCentripitalAcceleration;

  @JsonCreator
  public TrajectoryGenerationCubicComponent(
      @NotNull DriveUnidirectionalWithGyro drivetrain,
      double maxSpeedMeters,
      double maxAccelMeters,
      @Nullable Double maxCentripetalAcceleration,
      boolean reversed,
      @NotNull Pose2d startingPose,
      @NotNull List<Translation2d> translations,
      @NotNull Pose2d endingPose) {

    this.drivetrain = drivetrain;
    this.maxAccelMeters = maxAccelMeters;
    this.maxSpeedMeters = maxSpeedMeters;

    TrajectoryConstraint voltageConstraint =
        new DifferentialDriveVoltageConstraint(
            drivetrain.getLeftFeedforwardCalculator(), drivetrain.getDriveKinematics(), 12);

    // Create config for trajectory
    TrajectoryConfig configuration =
        new TrajectoryConfig(maxSpeedMeters, maxAccelMeters)
            .setKinematics(drivetrain.getDriveKinematics())
            .addConstraint(voltageConstraint)
            .setReversed(reversed);

    if (maxCentripetalAcceleration != null) {
      configuration.addConstraint(
          new CentripetalAccelerationConstraint(maxCentripetalAcceleration));
      this.maxCentripitalAcceleration = maxCentripetalAcceleration;
    }

    trajectory =
        TrajectoryGenerator.generateTrajectory(
            startingPose,
            translations,
            endingPose,
            configuration);
  }

  @Override
  public Trajectory getTrajectory() {
    return trajectory;
  }
}
