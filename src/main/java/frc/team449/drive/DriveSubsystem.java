package frc.team449.drive;

/** Any locomotion device for the robot. */
public interface DriveSubsystem {

  /** Completely stop the robot by setting the voltage to each side to be 0. */
  void fullStop();

  /** If this drive uses motors that can be disabled, enable them. */
  void enableMotors();

  /** Reset the position of the drive if it has encoders. */
  void resetPosition();
}
