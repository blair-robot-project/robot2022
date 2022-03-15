package frc.team449.drive.unidirectional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.team449.ahrs.AHRS;
import frc.team449.ahrs.SubsystemAHRS;
import frc.team449.motor.WrappedMotor;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

public class DriveUnidirectionalWithGyro extends DriveUnidirectionalBase implements SubsystemAHRS {
  /** The NavX gyro */
  @NotNull protected final AHRS ahrs;

  /** Drivetrain odometry tracker for tracking position */
  private final DifferentialDriveOdometry driveOdometry;

  /** Whether or not to use the NavX for driving straight */
  private boolean overrideGyro;

  /**
   * Default constructor.
   *
   * @param leftMaster The leader motor on the left side of the drive.
   * @param rightMaster The leader motor on the right side of the drive.
   * @param ahrs The NavX gyro for calculating this drive's heading and angular velocity.
   */
  public DriveUnidirectionalWithGyro(
      @NotNull WrappedMotor leftMaster,
      @NotNull WrappedMotor rightMaster,
      @NotNull AHRS ahrs,
      double trackWidth) {
    super(leftMaster, rightMaster, trackWidth);
    // Initialize stuff
    this.ahrs = ahrs;
    this.overrideGyro = false;
    this.driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(this.getHeading()));
  }

  /**
   * Create a {@link DriveUnidirectionalWithGyro} if not in a simulation, make a {@link
   * DriveUnidirectionalWithGyroSim} otherwise.
   *
   * @param driveSim The drivetrain simulation object, if needed
   * @param leftEncSim The simulated encoder for the left side, if needed
   * @param rightEncSim The simulated encoder for the left side, if needed
   */
  @Contract("_, _, _ -> new")
  public @NotNull DriveUnidirectionalWithGyro simIfNeeded(
      @NotNull DifferentialDrivetrainSim driveSim,
      @NotNull EncoderSim leftEncSim,
      @NotNull EncoderSim rightEncSim) {
    if (RobotBase.isReal()) {
      return this;
    } else {
      return new DriveUnidirectionalWithGyroSim(this, driveSim, leftEncSim, rightEncSim);
    }
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  /**
   * Get the robot's heading using the AHRS
   *
   * @return robot heading, in degrees, on [-180, 180]
   */
  @Override
  public double getHeading() {
    return this.ahrs.getHeading();
  }

  /**
   * Set the robot's heading.
   *
   * @param heading The heading to set to, in degrees on [-180, 180].
   */
  @Override
  public void setHeading(final double heading) {
    this.ahrs.setHeading(heading);
  }

  /**
   * Get the robot's cached heading.
   *
   * @return robot heading, in degrees, on [-180, 180].
   */
  @Override
  public double getHeadingCached() {
    return this.ahrs.getCachedHeading();
  }

  /**
   * Get the robot's angular velocity.
   *
   * @return Angular velocity in degrees/sec
   */
  @Override
  public double getAngularVel() {
    return this.ahrs.getAngularVelocity();
  }

  /**
   * Get the robot's cached angular velocity.
   *
   * @return Angular velocity in degrees/sec
   */
  @Override
  public double getAngularVelCached() {
    return this.ahrs.getCachedAngularVelocity();
  }

  /**
   * Get the robot's angular displacement since being turned on.
   *
   * @return Angular displacement in degrees.
   */
  @Override
  public double getAngularDisplacement() {
    return this.ahrs.getAngularDisplacement();
  }

  /**
   * Get the robot's cached angular displacement since being turned on.
   *
   * @return Angular displacement in degrees.
   */
  @Override
  public double getAngularDisplacementCached() {
    return this.ahrs.getCachedAngularDisplacement();
  }

  /**
   * Get the pitch value.
   *
   * @return The pitch, in degrees from [-180, 180]
   */
  @Override
  public double getPitch() {
    return this.ahrs.getPitch();
  }

  /**
   * Get the cached pitch value.
   *
   * @return The pitch, in degrees from [-180, 180]
   */
  @Override
  public double getCachedPitch() {
    return this.ahrs.getCachedPitch();
  }

  /**
   * @return true if the NavX is currently overriden, false otherwise.
   */
  @Override
  @Log
  public boolean getOverrideGyro() {
    return this.overrideGyro;
  }

  /**
   * @param override true to override the NavX, false to un-override it.
   */
  @Override
  public void setOverrideGyro(final boolean override) {
    this.overrideGyro = override;
  }

  /** Reset odometry tracker to current robot pose */
  @Log
  public void resetOdometry(final Pose2d pose) {
    resetPosition();
    ahrs.setHeading(pose.getRotation().getDegrees());
    driveOdometry.resetPosition(pose, Rotation2d.fromDegrees(this.getHeading()));
  }

  /** Update odometry tracker with current heading, and encoder readings */
  public void updateOdometry() {
    // need to convert to meters
    this.driveOdometry.update(
        Rotation2d.fromDegrees(this.getHeading()), this.getLeftPos(), this.getRightPos());
  }

  /**
   * @return Current estimated pose based on odometry tracker data
   */
  @Log.ToString
  @NotNull
  public Pose2d getCurrentPose() {
    return this.driveOdometry.getPoseMeters() != null
        ? this.driveOdometry.getPoseMeters()
        : new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
  }

  /**
   * @return Current wheel speeds based on encoder readings for future pose correction
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // need to convert to meters
    return new DifferentialDriveWheelSpeeds(this.getLeftVel(), this.getRightVel());
  }

  @Override
  public String getName() {
    return this.getClass().getSimpleName()
        + "_"
        + this.leftMaster.configureLogName()
        + "_"
        + this.rightMaster.configureLogName();
  }

  @Override
  public String configureLogName() {
    return this.getName();
  }
}
