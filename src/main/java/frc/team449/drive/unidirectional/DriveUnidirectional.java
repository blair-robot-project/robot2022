package frc.team449.drive.unidirectional;

import frc.team449.drive.DriveSubsystem;

/**
 * A drive with a left side and a right side. "Unidirectional" because it can only move forwards or
 * backwards, not sideways.
 */
public interface DriveUnidirectional extends DriveSubsystem {

  /**
   * Set the output of each side of the drive.
   *
   * @param left The output for the left side of the drive, from [-1, 1]
   * @param right the output for the right side of the drive, from [-1, 1]
   */
  void setOutput(double left, double right);

  /**
   * Get the velocity of the left side of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  double getLeftVel();

  /**
   * Get the velocity of the right side of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  double getRightVel();

  /**
   * Get the position of the left side of the drive.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  double getLeftPos();

  /**
   * Get the position of the right side of the drive.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  double getRightPos();

  /**
   * Get the cached velocity of the left side of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  double getLeftVelCached();

  /**
   * Get the cached velocity of the right side of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  double getRightVelCached();

  /**
   * Get the cached position of the left side of the drive.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  double getLeftPosCached();

  /**
   * Get the cached position of the right side of the drive.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  double getRightPosCached();
}
