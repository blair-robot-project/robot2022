package frc.team449.robot2022.cargo;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team449.oi.joystick.RumbleComponent;
import frc.team449.wrappers.Limelight;

/** Rumble the intake joystick if Limelight detects a ball of the opposite color */
public class IntakeLimelightRumbleComponent implements RumbleComponent {
  private final Limelight limelight;
  private final int bluePipeline;
  private final int redPipeline;

  /**
   * @param bluePipeline Pipeline for detecting blue balls
   * @param redPipeline Pipeline for detecting red balls
   */
  public IntakeLimelightRumbleComponent(Limelight limelight, int bluePipeline, int redPipeline) {
    this.limelight = limelight;
    this.bluePipeline = bluePipeline;
    this.redPipeline = redPipeline;
  }

  @Override
  public void initialize() {
    var isBlue = DriverStation.getAlliance() == DriverStation.Alliance.Blue;
    // Set it to the pipeline to detect balls of the opposite color
    if (isBlue) {
      limelight.setPipeline(redPipeline);
    } else {
      limelight.setPipeline(bluePipeline);
    }
  }

  @Override
  public double maxOutput() {
    return 1;
  }

  @Override
  public Pair<Double, Double> getOutput() {
    var output = limelight.hasTarget() ? 1.0 : 0.0;
    return new Pair<>(output, output);
  }
}
