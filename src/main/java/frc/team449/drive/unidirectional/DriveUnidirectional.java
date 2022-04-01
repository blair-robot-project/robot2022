package frc.team449.drive.unidirectional;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
import edu.wpi.first.wpilibj.RobotController;
import frc.team449.drive.DriveSubsystem;
import frc.team449.generalInterfaces.updatable.Updatable;
import org.jetbrains.annotations.Nullable;

/**
 * A drive with a left side and a right side. "Unidirectional" because it can only move forwards or
 * backwards, not sideways.
 */
public interface DriveUnidirectional extends DriveSubsystem, Updatable {

  /**
   * Set the output of each side of the drive.
   *
   * @param left The output for the left side of the drive, from [-1, 1]
   * @param right the output for the right side of the drive, from [-1, 1]
   */
  void setOutput(double left, double right);

  /** Set the voltage of each side of the drive */
  default void setVoltage(double left, double right) {
    setOutput(
        left / RobotController.getBatteryVoltage(), right / RobotController.getBatteryVoltage());
  }

  /**
   * Get the velocity of the left side of the drive.
   *
   * @return The signed velocity in meters per second
   */
  double getLeftVel();

  /**
   * Get the velocity of the right side of the drive.
   *
   * @return The signed velocity in meters per second
   */
  double getRightVel();

  /**
   * Get the position of the left side of the drive.
   *
   * @return The signed position in meters
   */
  double getLeftPos();

  /**
   * Get the position of the right side of the drive.
   *
   * @return The signed position in meters
   */
  double getRightPos();

  /**
   * Get the cached velocity of the left side of the drive.
   *
   * @return The signed velocity in meters per second
   */
  @Nullable
  Double getLeftVelCached();

  /**
   * Get the cached velocity of the right side of the drive.
   *
   * @return The signed velocity in meters per second
   */
  @Nullable
  Double getRightVelCached();

  /**
   * Get the cached position of the left side of the drive.
   *
   * @return The signed position in meters
   */
  double getLeftPosCached();

  /**
   * Get the cached position of the right side of the drive.
   *
   * @return The signed position in meters
   */
  @Nullable
  Double getRightPosCached();
}
