package frc.team449.drive.unidirectional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.team449.ahrs.AHRS;
import frc.team449.ahrs.SubsystemAHRS;
import frc.team449.drive.DriveSettings;
import frc.team449.motor.WrappedMotor;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

/** A drive with a cluster of any number of motors on each side. */
public class DriveUnidirectionalWithGyro extends DriveUnidirectionalBase implements SubsystemAHRS {
  /** The NavX gyro */
  @NotNull protected final AHRS ahrs;

  /** Drivetrain kinematics processor for measuring individual wheel speeds */
  @NotNull private final DifferentialDriveKinematics driveKinematics;

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
   * @param driveSettings The settings for this drivetrain
   */
  public DriveUnidirectionalWithGyro(
      @NotNull WrappedMotor leftMaster,
      @NotNull WrappedMotor rightMaster,
      @NotNull AHRS ahrs,
      @NotNull DriveSettings driveSettings) {
    super(leftMaster, rightMaster, driveSettings);
    // Initialize stuff
    this.ahrs = ahrs;
    this.overrideGyro = false;
    this.driveKinematics = new DifferentialDriveKinematics(driveSettings.trackWidth);
    this.driveOdometry = new DifferentialDriveOdometry(this.ahrs.getRotation());
  }

  /**
   * Create a {@link DriveUnidirectionalWithGyro} if not in a simulation, make a {@link
   * DriveUnidirectionalWithGyroSim} otherwise.
   *
   * @param leftMaster The leader motor on the left side of the drive.
   * @param rightMaster The leader motor on the right side of the drive.
   * @param ahrs The NavX gyro for calculating this drive's heading and angular velocity.
   * @param settings The settings for this drivetrain
   * @param driveSim The drivetrain simulation object, if needed
   * @param leftEncSim The simulated encoder for the left side, if needed
   * @param rightEncSim The simulated encoder for the left side, if needed
   */
  @Contract("_, _, _, _, _, _, _ -> new")
  public static @NotNull DriveUnidirectionalWithGyro createRealOrSim(
      @NotNull WrappedMotor leftMaster,
      @NotNull WrappedMotor rightMaster,
      @NotNull AHRS ahrs,
      @NotNull DriveSettings settings,
      @NotNull DifferentialDrivetrainSim driveSim,
      @NotNull EncoderSim leftEncSim,
      @NotNull EncoderSim rightEncSim) {
    if (RobotBase.isReal()) {
      return new DriveUnidirectionalWithGyro(leftMaster, rightMaster, ahrs, settings);
    } else {
      return new DriveUnidirectionalWithGyroSim(
          leftMaster, rightMaster, ahrs, settings, driveSim, leftEncSim, rightEncSim);
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    updateOdometry();
  }

  /**
   * Set voltage output raw
   *
   * @param left The voltage output for the left side of the drive from [-12, 12]
   * @param right The voltage output for the right side of the drive from [-12, 12]
   */
  public void setVoltage(final double left, final double right) {
    leftMaster.setVoltage(left);
    rightMaster.setVoltage(right);
  }

  @NotNull
  @Override
  public AHRS getAHRS() {
    return ahrs;
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
    resetEncoders();
    driveOdometry.resetPosition(pose, this.ahrs.getRotation());
  }

  /** Update odometry tracker with current heading, and encoder readings */
  public void updateOdometry() {
    this.driveOdometry.update(this.ahrs.getRotation(), this.getLeftPos(), this.getRightPos());
  }

  /**
   * @return Current estimated pose based on odometry tracker data
   */
  @Log.ToString
  @NotNull
  public Pose2d getCurrentPose() {
    return this.driveOdometry.getPoseMeters() != null
        ? this.driveOdometry.getPoseMeters()
        : new Pose2d();
  }

  /**
   * @return Current wheel speeds based on encoder readings for future pose correction
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // need to convert to meters
    return new DifferentialDriveWheelSpeeds(this.getLeftVel(), this.getRightVel());
  }

  /**
   * @return Kinematics processor for wheel speeds
   */
  @NotNull
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
