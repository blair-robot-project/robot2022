package frc.team449.robot2022.routines;

public final class AutoConstants {

  /**
   * How long to wait before intaking. Used to let the spitter finish spitting balls from the last
   * trajectory.
   */
  public static final double PAUSE_BEFORE_INTAKE = 0.2;
  /** How long to wait for the balls to be spat out */
  public static final double PAUSE_AFTER_SPIT = 0.2;
  /** How long to wait for a ball to be shot out */
  public static final double PAUSE_AFTER_SHOOT = .8;
  // Speeds
  public static final double AUTO_MAX_SPEED = 4.0,
      AUTO_MAX_ACCEL = 2.3,
      AUTO_MAX_CENTRIPETAL_ACCEL = 2;
  /** How long to wait for the robot to finish turning in place */
  public static final double TURN_TIMEOUT = 1.3;

  public static final double TURN_KP = 0.006, TURN_KI = 0, TURN_KD = 0.001;

  private AutoConstants() {}
}
