package frc.team449.robot2022.routines;

public final class AutoConstants {

  /**
   * How long to wait before intaking. Used to let the spitter finish spitting balls from the last
   * trajectory.
   */
  public static final double PAUSE_BEFORE_INTAKE = 0.2;
  /** How long to wait for the balls to be spat out */
  public static final double PAUSE_AFTER_SPIT = 0.2;
  // Speeds
  public static final double AUTO_MAX_SPEED = 3,
      AUTO_MAX_ACCEL = 1.0,
      AUTO_MAX_CENTRIPETAL_ACCEL = 2.0;

  private AutoConstants() {}
}
