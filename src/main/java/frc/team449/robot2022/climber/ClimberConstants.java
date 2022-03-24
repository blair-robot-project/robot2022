package frc.team449.robot2022.climber;

public final class ClimberConstants {
  /** Motor ports */
  public static final int RIGHT_CLIMBER_MOTOR_PORT = 6, LEFT_CLIMBER_MOTOR_PORT = 5;
  /** Solenoid channels */
  public static final int CLIMBER_PISTON_FWD_CHANNEL = 1, CLIMBER_PISTON_REV_CHANNEL = 0;
  /** Hall Effect sensor channels */
  public static final int CLIMBER_LEFT_SENSOR_CHANNEL = 0, CLIMBER_RIGHT_SENSOR_CHANNEL = 1;
  /** Speeds at which to extend and retract arms */
  public static final double CLIMBER_EXTEND_VEL = 0.4, CLIMBER_RETRACT_VEL = -0.5;
  /** Height limit during high climb */
  public static final double CLIMBER_DISTANCE = 1.8;
  /** Height limit during mid climb */
  public static final double CLIMBER_MID_DISTANCE = 1.67;
  /**
   * Height below which climber is considered to be at the bottom if hall effect sensor is activated
   */
  public static final double CLIMBER_DIFFERENTIATION_HEIGHT = 0.5;

  public static final double CLIMBER_FF_KS = 0,
      CLIMBER_FF_KV = 0,
      CLIMBER_FF_KA = 0,
      CLIMBER_FF_KG = 0;
  public static final double CLIMBER_LEFT_UPR = 0.239, // 0.1778,
      CLIMBER_RIGHT_UPR = 0.239; // 0.2286;
  // How close the arm gets to the limits before climber joystick starts rumbling
  public static final double CLIMBER_RUMBLE_TOLERANCE = 0.1;
  /** Whether the hall effect sensors are plugged in */
  public static final boolean CLIMBER_HAS_SENSORS = true;

  private ClimberConstants() {}
}
