package frc.team449.drive;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** Any locomotion device for the robot. */
public interface DriveSubsystem extends Subsystem {

  /** Completely stop the robot by setting the voltage to each side to be 0. */
  void fullStop();

  /** Reset the position of the drive if it has encoders. */
  void resetPosition();
}
