package frc.team449.wrappers;

import static com.kauailabs.navx.frc.AHRS.SerialDataType.kProcessedData;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.team449.generalInterfaces.updatable.Updatable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.Contract;

/** An invertible wrapper for the NavX. */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class AHRS implements Updatable, Loggable {

  /** The AHRS this class is a wrapper on. */
  protected final com.kauailabs.navx.frc.AHRS ahrs;

  /** A multiplier for the yaw angle. -1 to invert, 1 to not. */
  protected final int invertYaw;

  /** Cached values. */
  private double cachedHeading,
      cachedAngularDisplacement,
      cachedAngularVel,
      cachedXAccel,
      cachedYAccel,
      cachedPitch;

  /**
   * Default constructor.
   *
   * @param port The port the NavX is plugged into. It seems like only kMXP (the port on the RIO)
   *     works.
   * @param invertYaw Whether or not to invert the yaw axis. Defaults to true.
   */
  @JsonCreator
  public AHRS(@JsonProperty(required = true) final SerialPort.Port port, final Boolean invertYaw) {
    if (port.equals(SerialPort.Port.kMXP)) {
      this.ahrs = new com.kauailabs.navx.frc.AHRS(SPI.Port.kMXP);
    } else {
      this.ahrs = new com.kauailabs.navx.frc.AHRS(port, kProcessedData, (byte) 100);
    }
    setHeading(0);
    if (invertYaw == null || invertYaw) {
      this.invertYaw = -1;
    } else {
      this.invertYaw = 1;
    }
  }

  /**
   * Convert from gs (acceleration due to gravity) to meters/(second^2).
   *
   * @param accelGs An acceleration in gs.
   * @return That acceleration in meters/(sec^2)
   */
  @Contract(pure = true)
  protected static double gsToMetersPerSecondSquared(final double accelGs) {
    return accelGs * 9.80665; // Google said so
  }

  /**
   * Get the current yaw value.
   *
   * @return The heading, in degrees from [-180, 180]
   */
  public double getHeading() {
    return invertYaw * ahrs.getYaw();
  }

  /**
   * Set the current yaw value.
   *
   * @param headingDegrees An angle in degrees, from [-180, 180], to set the heading to.
   */
  public void setHeading(final double headingDegrees) {
    ahrs.setAngleAdjustment(headingDegrees);
  }

  /**
   * Get the current total angular displacement. Differs from getHeading because it doesn't limit
   * angle.
   *
   * @return The angular displacement, in degrees.
   */
  public double getAngularDisplacement() {
    return ahrs.getAngle() * invertYaw;
  }

  /**
   * Get the current angular yaw velocity.
   *
   * @return The angular yaw velocity, in degrees/sec.
   */
  public double getAngularVelocity() {
    return ahrs.getRate() * invertYaw;
  }

  /**
   * Get the absolute X acceleration of the robot, relative to the field.
   *
   * @return Linear X acceleration, in meters/(sec^2)
   */
  public double getXAccel() {
    return gsToMetersPerSecondSquared(ahrs.getWorldLinearAccelX());
  }

  /**
   * Get the absolute Y acceleration of the robot, relative to the field.
   *
   * @return Linear Y acceleration, in meters/(sec^2)
   */
  public double getYAccel() {
    return gsToMetersPerSecondSquared(ahrs.getWorldLinearAccelY());
  }

  /**
   * Get the pitch value.
   *
   * @return The pitch, in degrees from [-180, 180]
   */
  public double getPitch() {
    return ahrs.getPitch();
  }

  /**
   * Get the cached yaw value.
   *
   * @return The heading, in degrees from [-180, 180]
   */
  @Log
  public double getCachedHeading() {
    return cachedHeading;
  }

  /**
   * Get the cached total angular displacement. Differs from getHeading because it doesn't limit
   * angle.
   *
   * @return The angular displacement, in degrees.
   */
  @Log
  public double getCachedAngularDisplacement() {
    return cachedAngularDisplacement;
  }

  /**
   * Get the cached angular yaw velocity.
   *
   * @return The angular yaw velocity, in degrees/sec.
   */
  @Log
  public double getCachedAngularVelocity() {
    return cachedAngularVel;
  }

  /**
   * Get the cached absolute X acceleration of the robot, relative to the field.
   *
   * @return Linear X acceleration, in meters/(sec^2)
   */
  @Log
  public double getCachedXAccel() {
    return cachedXAccel;
  }

  /**
   * Get the cached absolute Y acceleration of the robot, relative to the field.
   *
   * @return Linear Y acceleration, in meters/(sec^2)
   */
  @Log
  public double getCachedYAccel() {
    return cachedYAccel;
  }

  /**
   * Get the cached pitch value.
   *
   * @return The pitch, in degrees from [-180, 180]
   */
  @Log
  public double getCachedPitch() {
    return cachedPitch;
  }

  /** Updates all cached values with current ones. */
  @Override
  public void update() {
    cachedHeading = getHeading();
    cachedAngularDisplacement = getAngularDisplacement();
    cachedAngularVel = getAngularVelocity();
    cachedXAccel = getXAccel();
    cachedYAccel = getYAccel();
    cachedPitch = getPitch();
  }
}
