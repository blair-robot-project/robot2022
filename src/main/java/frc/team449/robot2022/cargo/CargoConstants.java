package frc.team449.robot2022.cargo;

public final class CargoConstants {
  public static final int INTAKE_LEADER_PORT = 8,
      INTAKE_FOLLOWER_PORT = 10,
      SPITTER_PORT = 9,
      FLYWHEEL_MOTOR_PORT = 12;
  public static final int INTAKE_PISTON_FWD_CHANNEL = 3, INTAKE_PISTON_REV_CHANNEL = 2;
  public static final int HOOD_PISTON_FWD_CHANNEL = 5, HOOD_PISTON_REV_CHANNEL = 4;

  public static final int INTAKE_CURR_LIM = 20;

  /** Spitter feedforward */
  public static final double SPITTER_KS = -0.15654, SPITTER_KV = .12658, SPITTER_KA = .017184; //0.171731 ks
  /** Shooter feedforward */
  public static final double SHOOTER_KS = -0.15654, SHOOTER_KV = .12658, SHOOTER_KA = .017184; //0.171731 ks

  /** How much is given to feeder and intake motors */
  public static final double FEEDER_OUTPUT = 0.75;
  /** How much is given to the spitter when actually spitting */
  public static final double SPITTER_SPIT_SPEED_RPS = 30;
  /** The speed of the spitter when actually intaking, in RPS */
  public static final double SPITTER_INTAKE_SPEED_RPS = 31;
  /** The speed of the spitter when shooting, in RPS */
  public static final double SPITTER_SHOOT_SPEED_RPS = 40;
  /** The speed of the shooter flywheel when shooting high, in RPS */
  public static final double SHOOTER_SPEED_RPS = 45;
  /** Seconds to wait for flywheel to reach target velocity when shooting high */
  public static final double SHOOT_HIGH_SPINUP_TIME = 10;
  /** How many seconds to reverse the intake before spinning up and shooting */
  public static final double REVERSE_BEFORE_SHOOT_TIME = .07;
  /* Speed of the intake while doing the high shooter sequence */
  public static final double INTAKE_SPEED_HIGH_SEQUENCE = 0.5;
  /* Tolerance for the speed (flywheel and spitter)*/
  public static final double SHOOTER_TOLERANCE = 2.0;

  private CargoConstants() {}
}
