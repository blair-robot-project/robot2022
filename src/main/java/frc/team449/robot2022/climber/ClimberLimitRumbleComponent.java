package frc.team449.robot2022.climber;

import edu.wpi.first.math.Pair;
import frc.team449.oi.joystick.RumbleComponent;
import org.jetbrains.annotations.NotNull;

/**
 * Rumble if the climber gets too close to the bottom or its height limit (within {@link
 * ClimberLimitRumbleComponent#rumbleTolerance rumbleTolerance}. Rumbling increases linearly,
 * starting at 0 and reaching {@code rumbleTolerance}.
 */
public class ClimberLimitRumbleComponent implements RumbleComponent {

  private final PivotingTelescopingClimber climber;
  /** How close the arms can get to the limits before the joystick starts rumbling */
  private final double rumbleTolerance;

  public ClimberLimitRumbleComponent(
      @NotNull PivotingTelescopingClimber climber, double rumbleTolerance) {
    this.climber = climber;
    this.rumbleTolerance = rumbleTolerance;
  }

  @Override
  public double maxOutput() {
    return rumbleTolerance;
  }

  @Override
  public Pair<Double, Double> getOutput() {
    var leftDist = climber.leftArm.getHeight();
    var rightDist = climber.rightArm.getHeight();

    // Check the height limit depending on whether it's in mid climb (stowed) or high
    // climb
    var topLimit = climber.isStowed() ? climber.midDistance : climber.distanceTopBottom;

    // Amount to rumble left arm by if reached the bottom
    var leftBottomRumble = Math.max(0, rumbleTolerance - leftDist);
    // Amount to rumble left arm by if reached the top
    var leftTopRumble = Math.max(0, rumbleTolerance - (topLimit - leftDist));
    var rightBottomRumble = Math.max(0, rumbleTolerance - rightDist);
    var rightTopRumble = Math.max(0, rumbleTolerance - (topLimit - rightDist));

    return new Pair<>(leftBottomRumble + leftTopRumble, rightBottomRumble + rightTopRumble);
  }
}
