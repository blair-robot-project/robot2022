package frc.team449.robot2022;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
  /** Drive motor ports */
  public static final int RIGHT_LEADER_PORT = 1,
      RIGHT_LEADER_FOLLOWER_1_PORT = 11,
      RIGHT_LEADER_FOLLOWER_2_PORT = 7,
      LEFT_LEADER_PORT = 2,
      LEFT_LEADER_FOLLOWER_1_PORT = 4,
      LEFT_LEADER_FOLLOWER_2_PORT = 3;

  /** External encoder ports */
  public static final int
      LEFT_EXTERNAL_FWD_PORT = 6,
      LEFT_EXTERNAL_REV_PORT = 7,
      RIGHT_EXTERNAL_FWD_PORT = 4,
      RIGHT_EXTERNAL_REV_PORT = 5;

  public static final double DRIVE_WHEEL_RADIUS = Units.inchesToMeters(2);
  public static final double DRIVE_GEARING = 5.86;
  public static final int NEO_ENCODER_CPR = 1, DRIVE_EXT_ENCODER_CPR = 256;
  public static final int DRIVE_CURRENT_LIM = 40;
  public static final double DRIVE_ENC_POS_THRESHOLD = 0.15, DRIVE_ENC_VEL_THRESHOLD = 0.1;
  public static final double DRIVE_UPR = 0.3021211527151539;
  public static final double DRIVE_KP_VEL = 2, // 27.2,
      DRIVE_KI_VEL = 0.0,
      DRIVE_KD_VEL = 0.0,
      DRIVE_FF_KS = 0.1908,
      DRIVE_FF_KV = 2.5406,
      DRIVE_FF_KA = 0.44982;
  // todo characterize again to get better angular gains
  public static final double DRIVE_ANGLE_FF_KS = 0.20112,
      DRIVE_ANGLE_FF_KV = 2, //171.58,
      DRIVE_ANGLE_FF_KA = 0.5, //22.755,
      DRIVE_ANGLE_KP = 0.006, // 221.18
      DRIVE_ANGLE_KI = 0,
      DRIVE_ANGLE_KD = 0.03;
  // old value from measuring from the outside of the wheel: 0.6492875
  // measuring from the inside of the wheel : .57785
  public static final double DRIVE_TRACK_WIDTH = 0.61401; // 0.6492875;
  // Ramping
  public static final double RAMP_INCREASE = 0.9, RAMP_DECREASE = 0.7;
}
