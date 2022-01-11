package frc.team449.components;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
import edu.wpi.first.math.trajectory.Trajectory;

/** TODO add some actual javadocs here */
@JsonTypeInfo(
    use = JsonTypeInfo.Id.CLASS,
    include = JsonTypeInfo.As.WRAPPER_OBJECT,
    property = "@class")
public interface TrajectoryGenerationComponent {

  Trajectory getTrajectory();
}
