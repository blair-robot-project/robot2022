package frc.team449.robot2022.cargo;

public final class CargoConstants {
  public static final int INTAKE_LEADER_PORT = 8,
      INTAKE_FOLLOWER_PORT = 10,
      SPITTER_PORT = 9,
      FLYWHEEL_MOTOR_PORT = 12;
  public static final int INTAKE_PISTON_FWD_CHANNEL = 3, INTAKE_PISTON_REV_CHANNEL = 2;
  public static final int HOOD_PISTON_FWD_CHANNEL = 5, HOOD_PISTON_REV_CHANNEL = 4;

  public static final int INTAKE_CURR_LIM = 20;

  /** How much is given to feeder and intake motors */
  public static final double FEEDER_OUTPUT = 0.75;
  /** How much is given to the spitter when actually spitting */
  public static final double SPITTER_OUTPUT = 0.45;
  /** How much is given to the spitter when shooting */
  public static final double SPITTER_SHOOT_OUTPUT = 0.6;
  /** How much is given to the shooter flywheel when shooting high */
  public static final double SHOOTER_OUTPUT = 0.7;

  private CargoConstants() {}
}
