package frc.team449.drive.unidirectional;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.team449.generalInterfaces.DriveSettings;
import frc.team449.generalInterfaces.ahrs.SubsystemAHRS;
import frc.team449.wrappers.AHRS;
import frc.team449.wrappers.WrappedMotor;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

/** A drive with a cluster of any number of CANTalonSRX controlled motors on each side. */
@JsonTypeInfo(
    use = JsonTypeInfo.Id.CLASS,
    include = JsonTypeInfo.As.WRAPPER_OBJECT,
    property = "@class")
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class DriveUnidirectionalWithGyro extends DriveUnidirectionalBase implements SubsystemAHRS {
  /** The NavX gyro */
  @NotNull private final AHRS ahrs;

  /** Drivetrain kinematics processor for measuring individual wheel speeds */
  private final DifferentialDriveKinematics driveKinematics;

  /** Drivetrain odometry tracker for tracking position */
  private final DifferentialDriveOdometry driveOdometry;
  /** Whether or not to use the NavX for driving straight */
  private boolean overrideGyro;

  /**
   * Default constructor.
   *
   * @param leftMaster The master talon on the left side of the drive.
   * @param rightMaster The master talon on the right side of the drive.
   * @param ahrs The NavX gyro for calculating this drive's heading and angular velocity.
   * @param trackWidthMeters The width between the left and right wheels in meters
   */
  @JsonCreator
  public DriveUnidirectionalWithGyro(
      @NotNull WrappedMotor leftMaster,
      @NotNull WrappedMotor rightMaster,
      @NotNull AHRS ahrs,
      @NotNull DriveSettings driveSettings,
      double trackWidthMeters) {
    super(leftMaster, rightMaster, driveSettings);
    // Initialize stuff
    this.ahrs = ahrs;
    this.overrideGyro = false;
    this.driveKinematics = new DifferentialDriveKinematics(trackWidthMeters);
    this.driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(this.getHeading()));
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  /**
   * Set voltage output raw
   *
   * @param left The voltage output for the left side of the drive from [-12, 12]
   * @param right The voltage output for the right side of the drive from [-12, 12]
   */
  public void setVoltage(final double left, final double right) {
    leftMaster.setVoltage(left + settings.leftFeedforward.calculate(left));
    rightMaster.setVoltage(right + settings.rightFeedforward.calculate(right));
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

  /** @return true if the NavX is currently overriden, false otherwise. */
  @Override
  @Log
  public boolean getOverrideGyro() {
    return this.overrideGyro;
  }

  /** @param override true to override the NavX, false to un-override it. */
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

  /** @return Current estimated pose based on odometry tracker data */
  @Log.ToString
  @NotNull
  public Pose2d getCurrentPose() {
    return this.driveOdometry.getPoseMeters() != null
        ? this.driveOdometry.getPoseMeters()
        : new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
  }

  /** @return Current wheel speeds based on encoder readings for future pose correction */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // need to convert to meters
    return new DifferentialDriveWheelSpeeds(this.getLeftVel(), this.getRightVel());
  }

  /** @return Kinematics processor for wheel speeds */
  public DifferentialDriveKinematics getDriveKinematics() {
    return this.driveKinematics;
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
