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
  private final @NotNull DriveUnidirectionalWithGyro wrapped;
  private final @NotNull DifferentialDrivetrainSim driveSim;
  private final @NotNull EncoderSim leftEncSim;
  private final @NotNull EncoderSim rightEncSim;
  private double lastTime;

  /**
   * Create a simulated drive object. You may want to use {@link
   * DriveUnidirectionalWithGyro#simIfNeeded(DifferentialDrivetrainSim, EncoderSim, EncoderSim)}
   * instead
   *
   * @param wrapped The "real" drivetrain object this wraps around
   * @param driveSim The drivetrain simulation object, if needed
   * @param leftEncSim The simulated encoder for the left side, if needed
   * @param rightEncSim The simulated encoder for the left side, if needed
   */
  public DriveUnidirectionalWithGyroSim(
      @NotNull DriveUnidirectionalWithGyro wrapped,
      @NotNull DifferentialDrivetrainSim driveSim,
      @NotNull EncoderSim leftEncSim,
      @NotNull EncoderSim rightEncSim) {
    super(
        wrapped.leftMaster,
        wrapped.rightMaster,
        wrapped.ahrs,
        wrapped.getFeedforward(),
        wrapped.getDriveKinematics().trackWidthMeters);

    this.wrapped = wrapped;
    this.driveSim = driveSim;
    this.leftEncSim = leftEncSim;
    this.rightEncSim = rightEncSim;
    this.lastTime = Clock.currentTimeSeconds();
  }

  @Override
  public void setVoltage(double left, double right) {
    var leftRightVolts =
        feedforward.calculate(left, right, getLeftVelCached(), getRightVelCached());
    driveSim.setInputs(leftRightVolts.getFirst(), leftRightVolts.getSecond());
    super.setVoltage(leftRightVolts.getFirst(), leftRightVolts.getSecond());
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
