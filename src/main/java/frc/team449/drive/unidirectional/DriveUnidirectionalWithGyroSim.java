package frc.team449.drive.unidirectional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.team449.generalInterfaces.updatable.Updatable;
import frc.team449.other.Clock;
import org.jetbrains.annotations.NotNull;

public class DriveUnidirectionalWithGyroSim extends DriveUnidirectionalWithGyro
    implements Updatable {
  private final @NotNull DifferentialDrivetrainSim driveSim;
  private final @NotNull EncoderSim leftEncSim;
  private final @NotNull EncoderSim rightEncSim;
  private double lastTime;

  /**
   * Create a simulated drive object. You may want to use {@link
   * DriveUnidirectionalWithGyro#simIfNeeded(DifferentialDrivetrainSim, EncoderSim, EncoderSim)}
   * instead
   *
   * @param real The "real" drivetrain object this wraps around
   * @param driveSim The drivetrain simulation object, if needed
   * @param leftEncSim The simulated encoder for the left side, if needed
   * @param rightEncSim The simulated encoder for the left side, if needed
   */
  public DriveUnidirectionalWithGyroSim(
      @NotNull DriveUnidirectionalWithGyro real,
      @NotNull DifferentialDrivetrainSim driveSim,
      @NotNull EncoderSim leftEncSim,
      @NotNull EncoderSim rightEncSim) {
    super(real.leftMaster, real.rightMaster, real.ahrs, real.getDriveKinematics().trackWidthMeters);

    this.driveSim = driveSim;
    this.leftEncSim = leftEncSim;
    this.rightEncSim = rightEncSim;
    this.lastTime = Clock.currentTimeSeconds();
  }

  @Override
  public void setVoltage(double left, double right) {
    driveSim.setInputs(left, right);
    super.setVoltage(left, right);
  }

  @Override
  public void fullStop() {
    this.setVoltage(0, 0);
  }

  @Override
  public void disable() {
    this.setVoltage(0, 0);
  }

  @Override
  public void periodic() {
    this.update();
    super.periodic();
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    driveSim.setPose(pose);
    super.resetOdometry(pose);
  }

  @Override
  public @NotNull Pose2d getCurrentPose() {
    return driveSim.getPose();
  }

  @Override
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        driveSim.getLeftVelocityMetersPerSecond(), driveSim.getRightVelocityMetersPerSecond());
  }

  @Override
  public void update() {
    var currTime = Clock.currentTimeSeconds();
    driveSim.update(currTime - this.lastTime);
    leftEncSim.setDistance(driveSim.getLeftPositionMeters());
    leftEncSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    rightEncSim.setDistance(driveSim.getRightPositionMeters());
    rightEncSim.setRate(driveSim.getRightVelocityMetersPerSecond());
    ahrs.setHeading(driveSim.getHeading().getDegrees());

    this.lastTime = currTime;
  }
}
