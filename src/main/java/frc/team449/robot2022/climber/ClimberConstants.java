package frc.team449.robot2022.climber;

import edu.wpi.first.math.util.Units;

public final class ClimberConstants {
  /** Motor ports */
  public static final int RIGHT_CLIMBER_MOTOR_PORT = 6, LEFT_CLIMBER_MOTOR_PORT = 5;
  /** Solenoid channels */
  public static final int CLIMBER_PISTON_FWD_CHANNEL = 1, CLIMBER_PISTON_REV_CHANNEL = 0;
  /** Hall Effect sensor channels */
  public static final int CLIMBER_LEFT_SENSOR_CHANNEL = 0, CLIMBER_RIGHT_SENSOR_CHANNEL = 1;
  /** Percent output at which to extend and retract arms */
  public static final double CLIMBER_EXTEND_OUTPUT = 0.75,
      CLIMBER_RETRACT_OUTPUT = -0.55,
      CLIMBER_RETRACT_OUTPUT_SLOW = -0.3;
  /** Height limit during high climb */
  public static final double CLIMBER_DISTANCE = 1.8;
  /** Height limit during mid climb */
  public static final double CLIMBER_MID_DISTANCE = 1.67;
  /**
   * Height below which climber is considered to be at the bottom if hall effect sensor is activated
   */
  public static final double CLIMBER_DIFFERENTIATION_HEIGHT = 0.2;

  public static final double CLIMBER_FF_KS = 0,
      CLIMBER_FF_KV = 0,
      CLIMBER_FF_KA = 0,
      CLIMBER_FF_KG = 0;
  public static final double CLIMBER_LEFT_UPR = 0.239, // 0.1778,
      CLIMBER_RIGHT_UPR = 0.239; // 0.2286;

  public static final double CLIMBER_GEARING = 27;
  // How close the arm gets to the limits before climber joystick starts rumbling
  public static final double CLIMBER_RUMBLE_TOLERANCE = 0.1;
  /** Whether the hall effect sensors are plugged in */
  public static final boolean CLIMBER_HAS_SENSORS = true;
  public static final double CLIMBER_CARRIAGE_MASS = 10; //Units.lbsToKilograms(125) / 2;
  public static final double CLIMBER_WINCH_RADIUS = Units.inchesToMeters(5);
  /** Every meter gets this many units in the Mechanism2d widget */
  public static final double CLIMBER_MECH_SCALE = 20;

  private ClimberConstants() {}
}
