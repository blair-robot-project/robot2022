package frc.team449.drive.unidirectional;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.updatable.Updater;
import io.github.oblarg.oblog.Loggable;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/** A simple unidirectional drive with no encoders. */
public class DriveUnidirectionalSimple extends SubsystemBase
    implements DriveUnidirectional, Loggable {

  /** The motor for the left side of the drive. */
  @NotNull private final MotorController leftMotor;

  /** The motor for the right side of the drive. */
  @NotNull private final MotorController rightMotor;

  /**
   * Default constructor
   *
   * @param leftMotor The motor for the left side of the drive.
   * @param rightMotor The motor for the right side of the drive.
   */
  public DriveUnidirectionalSimple(
      @NotNull final MotorController leftMotor, @NotNull final MotorController rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    Updater.subscribe(this);
  }

  /**
   * Set the output of each side of the drive.
   *
   * @param left The output for the left side of the drive, from [-1, 1]
   * @param right the output for the right side of the drive, from [-1, 1]
   */
  @Override
  public void setOutput(final double left, final double right) {
    leftMotor.set(left);
    rightMotor.set(right);
  }

  /**
   * Get the velocity of the left side of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  @Nullable
  @Override
  public Double getLeftVel() {
    return null;
  }

  /**
   * Get the velocity of the right side of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  @Nullable
  @Override
  public Double getRightVel() {
    return null;
  }

  /**
   * Get the position of the left side of the drive.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  @Nullable
  @Override
  public Double getLeftPos() {
    return null;
  }

  /**
   * Get the position of the right side of the drive.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  @Nullable
  @Override
  public Double getRightPos() {
    return null;
  }

  /**
   * Get the cached velocity of the left side of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  @Nullable
  @Override
  public Double getLeftVelCached() {
    return null;
  }

  /**
   * Get the cached velocity of the right side of the drive.
   *
   * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
   */
  @Nullable
  @Override
  public Double getRightVelCached() {
    return null;
  }

  /**
   * Get the cached position of the left side of the drive.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  @Nullable
  @Override
  public Double getLeftPosCached() {
    return null;
  }

  /**
   * Get the cached position of the right side of the drive.
   *
   * @return The signed position in meters, or null if the drive doesn't have encoders.
   */
  @Nullable
  @Override
  public Double getRightPosCached() {
    return null;
  }

  /** Completely stop the robot by setting the voltage to each side to be 0. */
  @Override
  public void fullStop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  /** If this drive uses motors that can be disabled, enable them. */
  @Override
  public void enableMotors() {}

  /** Reset the position of the drive if it has encoders. */
  @Override
  public void resetPosition() {
    // No encoders, do nothing
  }

  /** Updates all cached values with current ones. */
  @Override
  public void update() {
    // Do nothing
  }
}
