package frc.team449.robot2022.cargo;

public final class CargoConstants {
  public static final int INTAKE_LEADER_PORT = 8,
      INTAKE_FOLLOWER_PORT = 10,
      SPITTER_PORT = 9,
      FLYWHEEL_MOTOR_PORT = 449; //Change later
  public static final int INTAKE_PISTON_FWD_CHANNEL = 3, INTAKE_PISTON_REV_CHANNEL = 2;
  public static final int INTAKE_CURR_LIM = 20;

  public static final double INTAKE_SPEED = 0.75, SPITTER_SPEED = 0.45;

  public static final double SHOOT_HIGH_OUTPUT = 0.7, SHOOT_LOW_OUTPUT = 0.45;

  private CargoConstants() {}
}
