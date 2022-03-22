package frc.team449.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.jetbrains.annotations.NotNull;

public final class DriveSettingsBuilder {
  private SimpleMotorFeedforward feedforward;
  private PIDController leftVelPID;
  private PIDController rightVelPID;
  private Double trackWidth;

  @NotNull
  public DriveSettings build() {
    return new DriveSettings(
        feedforward,
        leftVelPID,
        rightVelPID,
        trackWidth);
  }

  public DriveSettingsBuilder feedforward(SimpleMotorFeedforward feedforward) {
    this.feedforward = feedforward;
    return this;
  }

  public DriveSettingsBuilder leftVelPID(PIDController leftVelPID) {
    this.leftVelPID = leftVelPID;
    return this;
  }

  public DriveSettingsBuilder rightVelPID(PIDController rightVelPID) {
    this.rightVelPID = rightVelPID;
    return this;
  }

  public DriveSettingsBuilder trackWidth(double trackWidth) {
    this.trackWidth = trackWidth;
    return this;
  }
}
