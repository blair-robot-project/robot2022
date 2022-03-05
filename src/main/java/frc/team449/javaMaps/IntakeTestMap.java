package frc.team449.javaMaps;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449._2022robot.cargo.Cargo2022;
import frc.team449.ahrs.PIDAngleControllerBuilder;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.UnidirectionalNavXDefaultDrive;
import frc.team449.generalInterfaces.doubleUnaryOperator.Polynomial;
import frc.team449.generalInterfaces.doubleUnaryOperator.RampComponent;
import frc.team449.generalInterfaces.limelight.Limelight;
import frc.team449.javaMaps.builders.DriveSettingsBuilder;
import frc.team449.javaMaps.builders.RamseteBuilder;
import frc.team449.javaMaps.builders.SparkMaxConfig;
import frc.team449.javaMaps.builders.ThrottlePolynomialBuilder;
import frc.team449.oi.buttons.SimpleButton;
import frc.team449.oi.throttles.ThrottleSum;
import frc.team449.oi.throttles.ThrottleWithRamp;
import frc.team449.oi.unidirectional.arcade.OIArcadeWithDPad;
import frc.team449.other.Debouncer;
import frc.team449.other.FollowerUtils;
import frc.team449.other.Updater;
import frc.team449.wrappers.AHRS;
import frc.team449.wrappers.PDP;
import frc.team449.wrappers.RumbleableJoystick;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

public class IntakeTestMap {
  // Motor IDs
  public static final int RIGHT_LEADER_PORT = 1,
          RIGHT_LEADER_FOLLOWER_1_PORT = 11,
          RIGHT_LEADER_FOLLOWER_2_PORT = 7,
          LEFT_LEADER_PORT = 2,
          LEFT_LEADER_FOLLOWER_1_PORT = 4,
          LEFT_LEADER_FOLLOWER_2_PORT = 3,
          INTAKE_LEADER_PORT = 8,
          INTAKE_FOLLOWER_PORT = 10,
          SPITTER_PORT = 9,
          RIGHT_CLIMBER_MOTOR_PORT = 6,
          LEFT_CLIMBER_MOTOR_PORT = 5;
  /*
  public int chVEL = 1;
  public void changeVel() {
    if (chVEL == 1) {
      DRIVE_KP_VEL = 0.0001;
      chVEL = 2;
    }
    else {
      DRIVE_KP_VEL = 0.001;
      chVEL = 1;
    }
  }


  */
  // Other CAN IDs
  public static final int PDP_CAN = 1;
  // Controller ports
  public static final int MECHANISMS_JOYSTICK_PORT = 0, DRIVE_JOYSTICK_PORT = 1;
  // Limelight
  public static final int DRIVER_PIPELINE = 0; // TODO find out what this is!
  // Speeds
  public static final double INTAKE_SPEED = 0.4, SPITTER_SPEED = 0.5;
  public static final double AUTO_MAX_SPEED = 1.8; //0.05 for testing
  public static final double AUTO_MAX_ACCEL = .35; //0.05 for testing
  // Drive constants
  public static final double DRIVE_WHEEL_RADIUS = Units.inchesToMeters(2);
  public static final double DRIVE_GEARING = 5.86;
  public static final int DRIVE_CURRENT_LIM = 50;
  // old value from measuring from the outside of the wheel: 0.6492875
  // measuring from the inside of the wheel : .57785
  public static final double DRIVE_TRACK_WIDTH = 0.6492875;

  // Other constants
  public static final double CLIMBER_DISTANCE = 0.5;
  public static final double DRIVE_KP_VEL = 0.001,
          DRIVE_KD_VEL = 0, //0.00005,
          DRIVE_KP_POS = 45.269,
          DRIVE_KD_POS = 3264.2,
          DRIVE_FF_KS = 0.15084,
          DRIVE_FF_KV = 2.4303,
          DRIVE_FF_KA = 0.5323;
  //if changeVEL being used change from constant to changable variable
  private IntakeTestMap() {}

  @NotNull
  public static RobotMap createRobotMap() {
    var pdp = new PDP(PDP_CAN, new RunningLinRegComponent(250, 0.75), PowerDistribution.ModuleType.kCTRE);
    var mechanismsJoystick = new RumbleableJoystick(MECHANISMS_JOYSTICK_PORT);
    var driveJoystick = new RumbleableJoystick(DRIVE_JOYSTICK_PORT);

    var navx = new AHRS(SerialPort.Port.kMXP, true);

    // Widget to show robot pose+trajectory in Glass
    var field = new Field2d();
    SmartDashboard.putData(field);

    var limelight = new Limelight(DRIVER_PIPELINE);
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    // NavX expects clockwise positive, but sim outputs clockwise negative
    angle.set(180);

    var driveMasterPrototype =
            new SparkMaxConfig()
                    .setEnableBrakeMode(true)
                    .setUnitPerRotation(2 * Math.PI * DRIVE_WHEEL_RADIUS) // = 0.3191858136
                    .setCurrentLimit(DRIVE_CURRENT_LIM)
                    .setPostEncoderGearing(DRIVE_GEARING)
                    .setEnableVoltageComp(true)
                    .setCalculateVel(true); // TODO remove this as soon as possible
    var leftMaster =
            driveMasterPrototype
                    .copy()
                    .setPort(LEFT_LEADER_PORT)
                    .setName("left")
                    .setReverseOutput(false)
                    .addSlaveSpark(FollowerUtils.createFollowerSpark(LEFT_LEADER_FOLLOWER_1_PORT), false)
                    .addSlaveSpark(FollowerUtils.createFollowerSpark(LEFT_LEADER_FOLLOWER_2_PORT), false)
                    .createReal();
    var rightMaster =
            driveMasterPrototype
                    .copy()
                    .setName("right")
                    .setPort(RIGHT_LEADER_PORT)
                    .setReverseOutput(true)
                    .addSlaveSpark(FollowerUtils.createFollowerSpark(RIGHT_LEADER_FOLLOWER_1_PORT), false)
                    .addSlaveSpark(FollowerUtils.createFollowerSpark(RIGHT_LEADER_FOLLOWER_2_PORT), false)
                    .createReal();

    var drive =
            new DriveUnidirectionalWithGyro(
                    leftMaster,
                    rightMaster,
                    navx,
                    new DriveSettingsBuilder()
                            .feedforward(new SimpleMotorFeedforward(DRIVE_FF_KS, DRIVE_FF_KV, DRIVE_FF_KA))
                            .trackWidth(DRIVE_TRACK_WIDTH)
                            .build());

    var throttlePrototype =
            new ThrottlePolynomialBuilder().stick(driveJoystick).smoothingTimeSecs(0.06);
    var rotThrottle =
            throttlePrototype
                    .axis(XboxController.Axis.kLeftX.value)
                    .deadband(0.05)
                    .inverted(false)
                    .polynomial(new Polynomial(Map.of(1., 1.), null))
                    .build();
    var fwdThrottle =
            new ThrottleWithRamp(
                    new ThrottleSum(
                            Set.of(
                                    throttlePrototype
                                            .axis(XboxController.Axis.kLeftTrigger.value)
                                            .deadband(0.05)
                                            .inverted(true)
                                            .polynomial(new Polynomial(Map.of(1., 1.), null))
                                            .build(),
                                    throttlePrototype
                                            .axis(XboxController.Axis.kRightTrigger.value)
                                            .inverted(false)
                                            .build())),
                    new RampComponent(.6, .5));
    var oi =
            new OIArcadeWithDPad(
                    rotThrottle,
                    fwdThrottle,
                    0.1,
                    false,
                    driveJoystick,
                    new Polynomial(
                            Map.of(1., 1.), // Curvature
                            null),
                    .3,
                    false);

    var driveDefaultCmd =
        new UnidirectionalNavXDefaultDrive<>(
            3.0,
            new Debouncer(0.15),
            drive,
            oi,
            null,
            new PIDAngleControllerBuilder()
                .absoluteTolerance(0)
                .onTargetBuffer(new Debouncer(1.5))
                .minimumOutput(0)
                .maximumOutput(0.6)
                .loopTimeMillis(null)
                .deadband(2)
                .inverted(false)
                .pid(0.01, 0, 0.03)
                .build());

    Supplier<InstantCommand> resetDriveOdometry =
            () -> new InstantCommand(() -> drive.resetOdometry(new Pose2d()), drive);
    SmartDashboard.putData("Reset odometry", resetDriveOdometry.get());

    var cargo =
        new Cargo2022(
            new SparkMaxConfig()
                .setName("intakeMotor")
                .setPort(INTAKE_LEADER_PORT)
                .addSlaveSpark(FollowerUtils.createFollowerSpark(INTAKE_FOLLOWER_PORT), true)
                .createReal(),
            new SparkMaxConfig()
                .setName("spitterMotor")
                .setPort(SPITTER_PORT)
                .setEnableBrakeMode(false)
                .createReal(),
            new DoubleSolenoid(
                FullMap.PCM_MODULE,
                PneumaticsModuleType.CTREPCM,
                FullMap.INTAKE_PISTON_FWD_CHANNEL,
                FullMap.INTAKE_PISTON_REV_CHANNEL),
            INTAKE_SPEED,
            SPITTER_SPEED);
    Supplier<Command> runIntake =
            () ->
                    new InstantCommand(cargo::runIntake, cargo)
                            .andThen(new WaitCommand(3))
                            .andThen(cargo::stop);
    Supplier<Command> spit =
            () ->
                    new InstantCommand(cargo::spit, cargo).andThen(new WaitCommand(2)).andThen(cargo::stop);
//    var armPrototype =
//            driveMasterPrototype
//                    .copy()
//                    .setRevSoftLimit(0.)
//                    .setFwdSoftLimit(CLIMBER_DISTANCE)
//                    .setUnitPerRotation(0.1949)
//                    .setEnableBrakeMode(true);
//    var leftArm =
//            new ClimberArm(
//                    armPrototype
//                            .copy()
//                            .setName("climber_left")
//                            .setPort(LEFT_CLIMBER_MOTOR_PORT)
//                            .setPostEncoderGearing(10)
//                            .setReverseOutput(true)
//                            .createReal(),
//                    new PIDController(12, 0, 0),
//                    new ElevatorFeedforward(0, 0, 0));
//    var rightArm =
//            new ClimberArm(
//                    driveMasterPrototype
//                            .copy()
//                            .setName("climber_right")
//                            .setPort(RIGHT_CLIMBER_MOTOR_PORT)
//                            .setPostEncoderGearing(10)
//                            .setReverseOutput(false)
//                            .createReal(),
//                    new PIDController(12, 0, 0),
//                    new ElevatorFeedforward(0, 0, 0));
//    var climber = new PivotingTelescopingClimber(leftArm, rightArm, CLIMBER_DISTANCE);

    // PUT YOUR SUBSYSTEM IN HERE AFTER INITIALIZING IT
    var subsystems = List.<Subsystem>of(drive, cargo /*, climber*/);

    Updater.subscribe(() -> field.setRobotPose(drive.getCurrentPose()));

//     Button bindings here
//     Take in balls but don't shoot
        new SimpleButton(mechanismsJoystick, XboxController.Button.kLeftBumper.value)
            .whileHeld(cargo::runIntake, cargo)
            .whenReleased(cargo::stop, cargo);
//     Run all motors in intake to spit balls out
        new SimpleButton(mechanismsJoystick, XboxController.Button.kRightBumper.value)
            .whileHeld(cargo::spit, cargo)
            .whenReleased(cargo::stop, cargo);

    // TODO BUTTON BINDINGS HERE
    //    new JoystickButton(mechanismsJoystick, XboxController.Button.kA.value)
    //            .whileActiveContinuous(
    //                    new WaitCommand(0.01)
    //                            .andThen(() -> climber.setSetpoint(climber.getSetpoint() + 0.01),
    // climber));
    //    new JoystickButton(mechanismsJoystick, XboxController.Button.kB.value)
    //            .whileActiveContinuous(
    //                    new WaitCommand(0.01)
    //                            .andThen(() -> climber.setSetpoint(climber.getSetpoint() - 0.01),
    // climber));

    var ramsetePrototype =
        new RamseteBuilder()
            .drivetrain(drive)
            .leftPid(new PIDController(DRIVE_KP_VEL, 0, DRIVE_KD_VEL))
            .rightPid(new PIDController(DRIVE_KP_VEL, 0, DRIVE_KD_VEL))
            .field(field);
    //
    //    var sCurve =
    //        spit.get()
    //            .andThen(cargo::runIntake, cargo)
    //
    // .andThen(ramsetePrototype.copy().name("scurvetraj").traj(sCurveTraj(drive)).build())
    //            .andThen(spit.get());

    // (assume blue alliance)
    // Start at bottom next to hub, shoot preloaded ball, then get the two balls in that region and
    // score those
    var scoreThenGetTwoThenScore =
            new InstantCommand(cargo::runIntake, cargo)
                    .andThen(
                            ramsetePrototype
                                    .copy()
                                    .name("1-2ballbluebottom")
                                    .traj(loadPathPlannerTraj("New Path"))
                                    .build())
                    .andThen(spit.get());

    // (assume blue alliance)
    // Start at bottom on edge of tape, get one ball, score that and the preloaded one, go back for
    // another ball, then score that
    //    var getOneThenScoreThenGetAnotherThenScore =
    //        new InstantCommand(cargo::runIntake, cargo)
    //            .andThen(
    //                ramsetePrototype
    //                    .copy()
    //                    .name("2-1ballbluebottom")
    //                    .traj(loadPathPlannerTraj("2-1ball blue bottom"))
    //                    .build()
    //                    .alongWith(
    //                        new WaitCommand(10).andThen(spit.get()).andThen(cargo::runIntake,
    // cargo)))
    //            .andThen(spit.get());

    List<Command> autoStartupCommands = List.of(resetDriveOdometry.get(), scoreThenGetTwoThenScore);

    List<Command> robotStartupCommands = List.of();

    List<Command> teleopStartupCommands =
            List.of(
                    /*new InstantCommand(climber::enable),*/
                    new InstantCommand(() -> drive.setDefaultCommand(driveDefaultCmd)),
                    new InstantCommand(cargo::stop));

    List<Command> testStartupCommands = List.of();
    var allCommands =
            new CommandContainer(
                    robotStartupCommands, autoStartupCommands, teleopStartupCommands, testStartupCommands);

    return new RobotMap(subsystems, pdp, allCommands, false);
  }

  /** Generate a trajectory for the S-shaped curve we're using to test */
  @NotNull
  private static Trajectory sCurveTraj(@NotNull DriveUnidirectionalWithGyro drive) {
    var ballPos = new Pose2d(new Translation2d(2, 0), Rotation2d.fromDegrees(0));
    var fwdTraj =
            TrajectoryGenerator.generateTrajectory(
                    new Pose2d(), List.of(), ballPos, trajConfig(drive).setReversed(false));
    var revTraj =
            TrajectoryGenerator.generateTrajectory(
                    ballPos, List.of(), new Pose2d(), trajConfig(drive).setReversed(true));
    return fwdTraj.concatenate(revTraj);
  }

  private static Trajectory testTraj(@NotNull DriveUnidirectionalWithGyro drive) {
    return TrajectoryGenerator.generateTrajectory(
            new Pose2d(new Translation2d(7.68, 2.82), Rotation2d.fromDegrees(-111.63)),
            List.of(),
            new Pose2d(new Translation2d(6.4, 1.37), Rotation2d.fromDegrees(-143.13)),
            trajConfig(drive));
  }

  @NotNull
  private static TrajectoryConfig trajConfig(@NotNull DriveUnidirectionalWithGyro drive) {
    return new TrajectoryConfig(AUTO_MAX_SPEED, AUTO_MAX_ACCEL)
            .setKinematics(drive.getDriveKinematics())
            .addConstraint(
                    new DifferentialDriveVoltageConstraint(
                            drive.getFeedforward(), drive.getDriveKinematics(), 12));
  }

  @NotNull
  private static Trajectory loadPathPlannerTraj(@NotNull String trajName) {
    var traj = PathPlanner.loadPath(trajName, AUTO_MAX_SPEED, AUTO_MAX_ACCEL, false);
    if (traj == null) {
      throw new Error("Trajectory not found: " + trajName);
    }
    return traj;
  }
}