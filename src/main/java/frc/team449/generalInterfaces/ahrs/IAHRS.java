package frc.team449.generalInterfaces.ahrs;

import frc.team449.generalInterfaces.updatable.Updatable;
import org.jetbrains.annotations.Contract;

/**
 * An gyro with the methods we need. This exists as a separate interface mainly to allow simulated
 * gyros to work.
 *
 * @see com.kauailabs.navx.frc.AHRS
 */
public interface IAHRS extends Updatable {
  /**
   * Convert from gs (acceleration due to gravity) to meters/(second^2).
   *
   * @param accelGs An acceleration in gs.
   * @return That acceleration in meters/(sec^2)
   */
  @Contract(pure = true)
  static double gsToMetersPerSecondSquared(final double accelGs) {
    return accelGs * 9.80665; // Google said so
  }

  /**
   * Get the current yaw value.
   *
   * @return The heading, in degrees from [-180, 180]
   */
  double getHeading();

  /**
   * Set the current yaw value.
   *
   * @param headingDegrees An angle in degrees, from [-180, 180], to set the heading to.
   */
  void setHeading(double headingDegrees);

  /**
   * Get the current total angular displacement. Differs from getHeading because it doesn't limit
   * angle.
   *
   * @return The angular displacement, in degrees.
   */
  double getAngularDisplacement();

  /**
   * Get the current angular yaw velocity.
   *
   * @return The angular yaw velocity, in degrees/sec.
   */
  double getAngularVelocity();

  /**
   * Get the absolute X acceleration of the robot, relative to the field.
   *
   * @return Linear X acceleration, in meters/(sec^2)
   */
  double getXAccel();

  /**
   * Get the absolute Y acceleration of the robot, relative to the field.
   *
   * @return Linear Y acceleration, in meters/(sec^2)
   */
  double getYAccel();

  /**
   * Get the pitch value.
   *
   * @return The pitch, in degrees from [-180, 180]
   */
  double getPitch();
}
