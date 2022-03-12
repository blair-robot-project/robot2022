package frc.team449._2022robot.cargo;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.generalInterfaces.limelight.Limelight;

public class IntakeLimelightCommand extends CommandBase {
  private final Cargo2022 cargo;
  private final Limelight limelight;
  private final int bluePipeline;
  private final int redPipeline;

  /**
   *
   * @param cargo
   * @param limelight
   * @param bluePipeline Pipeline for detecting blue balls
   * @param redPipeline Pipeline for detecting red balls
   */
  public IntakeLimelightCommand(Cargo2022 cargo, Limelight limelight, int bluePipeline, int redPipeline) {
    addRequirements(cargo);
    this.cargo = cargo;
    this.limelight = limelight;
    this.bluePipeline = bluePipeline;
    this.redPipeline = redPipeline;
  }

  @Override
  public void initialize() {
    var isBlue = DriverStation.getAlliance() == DriverStation.Alliance.Blue;
    if (isBlue) {
      limelight.setPipeline(redPipeline);
    } else {
      limelight.setPipeline(bluePipeline);
    }
  }

  @Override
  public void execute() {
    if (limelight.hasTarget()) {
      cargo.stop();
    } else {
      cargo.runIntake();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }
}
