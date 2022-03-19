package frc.team449.javaMaps;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449.ahrs.AHRS;
import frc.team449.ahrs.PIDAngleControllerBuilder;
import frc.team449.auto.builders.RamseteBuilder;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.drive.DriveSettingsBuilder;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.AHRS.NavXTurnToAngle;
import frc.team449.drive.unidirectional.commands.DriveAtSpeed;
import frc.team449.drive.unidirectional.commands.UnidirectionalNavXDefaultDrive;
import frc.team449.generalInterfaces.doubleUnaryOperator.Polynomial;
import frc.team449.generalInterfaces.doubleUnaryOperator.RampComponent;
import frc.team449.wrappers.Limelight;
import frc.team449.motor.builder.SparkMaxConfig;
import frc.team449.oi.throttles.ThrottlePolynomialBuilder;
import frc.team449.oi.throttles.ThrottleSum;
import frc.team449.oi.throttles.ThrottleWithRamp;
import frc.team449.oi.unidirectional.arcade.OIArcadeWithDPad;
import frc.team449.other.Debouncer;
import frc.team449.other.FollowerUtils;
import frc.team449.other.Updater;
import frc.team449.robot2022.cargo.Cargo2022;
import frc.team449.robot2022.climber.ClimberArm;
import frc.team449.robot2022.climber.PivotingTelescopingClimber;
import frc.team449.robot2022.routines.ThreeBallAuto;
import frc.team449.wrappers.PDP;
import frc.team449.wrappers.RumbleableJoystick;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

public class FullMap {
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
      LEFT_CLIMBER_MOTOR_PORT = 5,
      LEFT_EXTERNAL_FWD_PORT = 6,
      LEFT_EXTERNAL_REV_PORT = 7,
      RIGHT_EXTERNAL_FWD_PORT = 4,
      RIGHT_EXTERNAL_REV_PORT = 5;

  // Other CAN IDs
  public static final int PDP_CAN = 1, PCM_MODULE = 0;
  // Controller ports
  public static final int CARGO_JOYSTICK_PORT = 0,
      DRIVE_JOYSTICK_PORT = 1,
      CLIMBER_JOYSTICK_PORT = 2;
  // Limelight
  public static final int DRIVER_PIPELINE = 0, BLUE_PIPELINE = 1, RED_PIPELINE = 2;
  // Speeds
  public static final double INTAKE_SPEED = 0.75, SPITTER_SPEED = 0.45;
  public static final double AUTO_MAX_SPEED = 2, AUTO_MAX_ACCEL = .4;
  // Drive constants
  public static final double DRIVE_WHEEL_RADIUS = Units.inchesToMeters(2);
  public static final double DRIVE_GEARING = 1; // 5.86;
  public static final int DRIVE_ENCODER_CPR = 256;
  public static final int DRIVE_CURRENT_LIM = 40;
  public static final double DRIVE_UPR = 0.3021211527151539;
  public static final double DRIVE_KP_VEL = .0, // 27.2,
      DRIVE_KI_VEL = 0.0,
      DRIVE_KD_VEL = 0.0,
      DRIVE_KP_POS = 45.269,
      DRIVE_KD_POS = 3264.2,
      DRIVE_FF_KS = 0.20835,
      DRIVE_FF_KV = 2.4401,
      DRIVE_FF_KA = 0.3523;
  // todo actually use these feedforward values
  public static final double DRIVE_ANGLE_FF_KS = 0.20112,
      DRIVE_ANGLE_FF_KV = 171.58,
      DRIVE_ANGLE_FF_KA = 22.755,
      DRIVE_ANGLE_KP = 0.006, // 221.18
      DRIVE_ANGLE_KI = 0,
      DRIVE_ANGLE_KD = 0.03;
  // old value from measuring from the outside of the wheel: 0.6492875
  // measuring from the inside of the wheel : .57785
  public static final double DRIVE_TRACK_WIDTH = 0.61401; // 0.6492875;
  // Ramping
  public static final double RAMP_INCREASE = 0.9, RAMP_DECREASE = 0.7;
  // Climber
  public static final int CLIMBER_PISTON_FWD_CHANNEL = 0, CLIMBER_PISTON_REV_CHANNEL = 1;
  // todo find out what the channel numbers are
  public static final int CLIMBER_SENSOR_CHANNEL = 0; // todo find out what this really is
  public static final double CLIMBER_MAX_VEL = 0.1, CLIMBER_MAX_ACCEL = 0.1;
  public static final double CLIMBER_EXTEND_VEL = 0.2, CLIMBER_RETRACT_VEL = -0.3;
  public static final double CLIMBER_DISTANCE = 0.7, CLIMBER_MID_DISTANCE = 0.5;
  public static final double CLIMBER_KP = 500; // 600;
  public static final double CLIMBER_FF_KS = 0,
      CLIMBER_FF_KV = 0,
      CLIMBER_FF_KA = 0,
      CLIMBER_FF_KG = 0;
  public static final double CLIMBER_LEFT_UPR = 0.239, // 0.1778,
      CLIMBER_RIGHT_UPR = 0.239; // 0.2286;

  // Intake
  public static final int INTAKE_PISTON_FWD_CHANNEL = 3, INTAKE_PISTON_REV_CHANNEL = 2;
  public static final int INTAKE_CURR_LIM = 20;

  private FullMap() {}

  @NotNull
  public static RobotMap createRobotMap() {
    var pdp =
        new PDP(PDP_CAN, new RunningLinRegComponent(250, 0.75), PowerDistribution.ModuleType.kCTRE);
    var cargoJoystick = new RumbleableJoystick(CARGO_JOYSTICK_PORT);
    var driveJoystick = new RumbleableJoystick(DRIVE_JOYSTICK_PORT);
    var climberJoystick = new RumbleableJoystick(CLIMBER_JOYSTICK_PORT);

    var navx = AHRS.createRealOrSim(SerialPort.Port.kMXP, true);

    // Widget to show robot pose+trajectory in Glass
    var field = new Field2d();
    SmartDashboard.putData(field);

    var limelight = new Limelight(DRIVER_PIPELINE);
    limelight.setStreamMode(Limelight.StreamMode.STANDARD);
    limelight.setLedMode(Limelight.LedMode.OFF);

    var driveMasterPrototype =
        new SparkMaxConfig()
            .setEnableBrakeMode(true)
            .setUnitPerRotation(DRIVE_UPR)
            .setCurrentLimit(DRIVE_CURRENT_LIM)
            .setPostEncoderGearing(DRIVE_GEARING)
            .setEncoderCPR(DRIVE_ENCODER_CPR)
            .setEnableVoltageComp(true);

    var driveSim =
        new DifferentialDrivetrainSim(
            LinearSystemId.identifyDrivetrainSystem(
                DRIVE_FF_KV, DRIVE_FF_KA, DRIVE_ANGLE_FF_KV, DRIVE_ANGLE_FF_KA, DRIVE_TRACK_WIDTH),
            DCMotor.getNEO(3),
            DRIVE_GEARING,
            DRIVE_TRACK_WIDTH,
            DRIVE_WHEEL_RADIUS,
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    var leftExtEnc = new Encoder(LEFT_EXTERNAL_FWD_PORT, LEFT_EXTERNAL_REV_PORT, false);
    var rightExtEnc = new Encoder(RIGHT_EXTERNAL_FWD_PORT, RIGHT_EXTERNAL_REV_PORT, true);
    var leftEncSim = new EncoderSim(leftExtEnc);
    var rightEncSim = new EncoderSim(rightExtEnc);

    var leftMaster =
        driveMasterPrototype
            .copy()
            .setPort(LEFT_LEADER_PORT)
            .setName("left")
            .setReverseOutput(false)
            .setExternalEncoder(leftExtEnc)
            .addSlaveSpark(FollowerUtils.createFollowerSpark(LEFT_LEADER_FOLLOWER_1_PORT), false)
            .addSlaveSpark(FollowerUtils.createFollowerSpark(LEFT_LEADER_FOLLOWER_2_PORT), false)
            .createRealOrSim(leftEncSim);
    var rightMaster =
        driveMasterPrototype
            .copy()
            .setName("right")
            .setPort(RIGHT_LEADER_PORT)
            .setReverseOutput(true)
            .setExternalEncoder(rightExtEnc)
            .addSlaveSpark(FollowerUtils.createFollowerSpark(RIGHT_LEADER_FOLLOWER_1_PORT), false)
            .addSlaveSpark(FollowerUtils.createFollowerSpark(RIGHT_LEADER_FOLLOWER_2_PORT), false)
            .createRealOrSim(rightEncSim);

    var driveFeedforward = new SimpleMotorFeedforward(DRIVE_FF_KS, DRIVE_FF_KV, DRIVE_FF_KA);
    var drive =
        DriveUnidirectionalWithGyro.createRealOrSim(
            leftMaster,
            rightMaster,
            navx,
            new DriveSettingsBuilder()
                .feedforward(driveFeedforward)
                .trackWidth(DRIVE_TRACK_WIDTH)
                .build(),
            driveSim,
            leftEncSim,
            rightEncSim);

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
            new RampComponent(RAMP_INCREASE, RAMP_DECREASE));
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

    var pidAngleControllerPrototype =
        new PIDAngleControllerBuilder()
            .absoluteTolerance(0.001)
            .onTargetBuffer(new Debouncer(1.5))
            .minimumOutput(0)
            .maximumOutput(0.6)
            .loopTimeMillis(null)
            .deadband(2)
            .inverted(false)
            .pid(DRIVE_ANGLE_KP, DRIVE_ANGLE_KI, DRIVE_ANGLE_KD);

    var driveDefaultCmd =
        new UnidirectionalNavXDefaultDrive<>(
            3.0, new Debouncer(0.15), drive, oi, null, pidAngleControllerPrototype.build());

    Supplier<InstantCommand> resetDriveOdometry =
        () -> new InstantCommand(() -> drive.resetOdometry(new Pose2d()), drive);
    SmartDashboard.putData("Reset odometry", resetDriveOdometry.get());

    var deployIntake =
        new DoubleSolenoid(
            PCM_MODULE,
            PneumaticsModuleType.CTREPCM,
            INTAKE_PISTON_FWD_CHANNEL,
            INTAKE_PISTON_REV_CHANNEL);

    var cargo =
        new Cargo2022(
            new SparkMaxConfig()
                .setName("intakeMotor")
                .setPort(INTAKE_LEADER_PORT)
                .setCurrentLimit(INTAKE_CURR_LIM)
                .addSlaveSpark(FollowerUtils.createFollowerSpark(INTAKE_FOLLOWER_PORT), true)
                .createReal(),
            new SparkMaxConfig()
                .setName("spitterMotor")
                .setPort(SPITTER_PORT)
                .setEnableBrakeMode(false)
                .createReal(),
            deployIntake,
            INTAKE_SPEED,
            SPITTER_SPEED);

    var armPrototype =
        new SparkMaxConfig()
            .setEnableVoltageComp(true)
            //            .setRevSoftLimit(0.)
            //            .setFwdSoftLimit(CLIMBER_DISTANCE)
            .setUnitPerRotation(0.1949)
            .setPostEncoderGearing(27)
            .setEnableBrakeMode(true);
    var leftArm =
        new ClimberArm(
            armPrototype
                .copy()
                .setName("climber_left")
                .setPort(LEFT_CLIMBER_MOTOR_PORT)
                .setUnitPerRotation(CLIMBER_LEFT_UPR)
                .setReverseOutput(true)
                .createReal(),
            new ProfiledPIDController(
                CLIMBER_KP,
                0,
                0,
                new TrapezoidProfile.Constraints(CLIMBER_MAX_VEL, CLIMBER_MAX_ACCEL)),
            new ElevatorFeedforward(CLIMBER_FF_KS, CLIMBER_FF_KG, CLIMBER_FF_KV, CLIMBER_FF_KA));
    var rightArm =
        new ClimberArm(
            armPrototype
                .copy()
                .setName("climber_right")
                .setPort(RIGHT_CLIMBER_MOTOR_PORT)
                .setUnitPerRotation(CLIMBER_RIGHT_UPR)
                // checked 3/6/22 by Matthew N
                .setReverseOutput(false)
                .createReal(),

            //            driveMasterPrototype
            //                .copy()
            //                .setName("climber_right")
            //                .setPort(RIGHT_CLIMBER_MOTOR_PORT)
            //                .setUnitPerRotation(0.2286)
            //                    //checked 3/6/22 by Matthew N
            //                .setReverseOutput(false)
            //                .createReal(),
            new ProfiledPIDController(
                CLIMBER_KP,
                0,
                0,
                new TrapezoidProfile.Constraints(CLIMBER_MAX_VEL, CLIMBER_MAX_ACCEL)),
            new ElevatorFeedforward(CLIMBER_FF_KS, CLIMBER_FF_KG, CLIMBER_FF_KV, CLIMBER_FF_KA));
    var pivotPiston =
        new DoubleSolenoid(
            PCM_MODULE,
            PneumaticsModuleType.CTREPCM,
            CLIMBER_PISTON_FWD_CHANNEL,
            CLIMBER_PISTON_REV_CHANNEL);
    var climber =
        new PivotingTelescopingClimber(
            leftArm,
            rightArm,
            pivotPiston,
            RobotBase.isReal() ? new DigitalInput(CLIMBER_SENSOR_CHANNEL)::get : () -> false,
            CLIMBER_DISTANCE,
            CLIMBER_MID_DISTANCE);

    // PUT YOUR SUBSYSTEM IN HERE AFTER INITIALIZING IT
    var subsystems = List.<Subsystem>of(drive, cargo, climber);

    SmartDashboard.putData("Intake deploy piston: ", new InstantCommand(cargo::deployIntake));
    SmartDashboard.putData("Intake retract piston: ", new InstantCommand(cargo::retractIntake));

    Updater.subscribe(() -> field.setRobotPose(drive.getCurrentPose()));

    // Button bindings here
    // Take in balls but don't shoot
    new JoystickButton(cargoJoystick, XboxController.Button.kLeftBumper.value)
        .whileHeld(cargo::runIntake, cargo)
        .whenReleased(cargo::stop, cargo);
    // Run all motors in intake to spit balls out
    new JoystickButton(cargoJoystick, XboxController.Button.kRightBumper.value)
        .whileHeld(cargo::spit, cargo)
        .whenReleased(cargo::stop, cargo);
    // Stow/retract intake
    new JoystickButton(cargoJoystick, XboxController.Button.kX.value)
        .whenPressed(cargo::retractIntake);
    // Deploy intake
    new JoystickButton(cargoJoystick, XboxController.Button.kA.value)
        .whileHeld(cargo::deployIntake, cargo)
        .whenReleased(cargo::stop, cargo);
    // Run intake backwards
    new JoystickButton(cargoJoystick, XboxController.Button.kB.value)
        .whileHeld(cargo::runIntakeReverse, cargo)
        .whenReleased(cargo::stop);
    // Extend Climber
    new JoystickButton(climberJoystick, XboxController.Button.kY.value)
        .whileHeld(() -> climber.set(CLIMBER_EXTEND_VEL), climber)
        .whenReleased(() -> climber.set(0), climber);
    // Retract climber
    new JoystickButton(climberJoystick, XboxController.Button.kA.value)
        .whileHeld(() -> climber.set(CLIMBER_RETRACT_VEL), climber)
        .whenReleased(() -> climber.set(0), climber);
    // Pivot climber out
    new JoystickButton(climberJoystick, XboxController.Button.kB.value)
        .whenPressed(
            new InstantCommand(cargo::deployIntake).andThen(climber::pivotTelescopingArmOut));
    // Retract climber arm in with piston
    new JoystickButton(climberJoystick, XboxController.Button.kX.value)
        .whenPressed(climber::pivotTelescopingArmIn);

    var ramsetePrototype =
        new RamseteBuilder()
            .drivetrain(drive)
            .pidController(new PIDController(DRIVE_KP_VEL, DRIVE_KI_VEL, DRIVE_KD_VEL))
            //            .b(2.25)
            //            .zeta(0.6)
            .anglePID(
                new PIDAngleControllerBuilder()
                    .absoluteTolerance(0.5)
                    .onTargetBuffer(null)
                    .minimumOutput(0)
                    .maximumOutput(null)
                    .loopTimeMillis(20)
                    .deadband(0.05)
                    .inverted(false)
                    .pid(0.002, 0, 0)
                    .build())
            .angleTimeout(0)
            .field(null);
    // .field(field);

    Supplier<Command> spit =
        () ->
            new InstantCommand(cargo::spit, cargo)
                .andThen(new WaitCommand(1))
                .andThen(cargo::stop, cargo);

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

    // Spit the preloaded ball, pick up another, come back and spit it out
    var topOneOneTraj =
        oneThenOneBallTraj(
            drive,
            cargo,
            ramsetePrototype.name("topOneOne"),
            pose(7.11, 4.80, 159.25),
            pose(5.39, 5.91, 137.86));
    var midOneOneTraj =
        oneThenOneBallTraj(
            drive,
            cargo,
            ramsetePrototype.name("midOneOne"),
            pose(7.56, 3.00, -111.80),
            pose(5.58, 2.00, -173.42));
    var bottomOneOneTraj =
        oneThenOneBallTraj(
            drive,
            cargo,
            ramsetePrototype.copy().name("bottomOneOne"),
            pose(8.01, 2.82, -111.80),
            pose(7.67, 0.77, -91.97));

    //    // Start at the edge at the top, collect the top ball, then come back and spit
    //    var hangarTwoBallTraj =
    //        twoBallTraj(
    //            drive,
    //            cargo,
    //            ramsetePrototype.copy().name("hangar_two_ball_auto").field(field).angleTimeout(0),
    //            pose(6.06, 5.13, 136.40),
    //            pose(5.33, 5.87, 134.37),
    //            pose(6.76, 4.05, 180 - 41));
    var hangarTwoBallTraj =
        twoBallTraj(
            drive,
            cargo,
            ramsetePrototype.copy().name("hangar_two_ball_auto").field(field).angleTimeout(0),
            pose(6.06, 5.13, 136.40),
            pose(5.33, 5.87, 134.37),
            pose(6.76, 4.05, 180 - 41));
    //    // Start at the edge at the bottom, collect the bottom ball, then come back and spit
    var stationTwoBallTraj =
        twoBallTraj(
            drive,
            cargo,
            ramsetePrototype.name("station_two_ball_auto"),
            pose(7.55, 1.83, -88.32),
            pose(7.63, 0.76, -86.19),
            pose(7.83, 2.91, 180 + 68.63));
    var ballX = 7.65;
    var ballY = 0.69;
    var ballAngle = -92.44;
    var turnAngle = 170.0;
    var ballX2 = 5.55;
    var ballY2 = 1.85;
    var turnAngle2 = 200;
    var hubPose = pose(7.43, 2.83, -180 + 46.64);
    var threeBallAuto =
        new InstantCommand(cargo::spit, cargo)
            .andThen(new WaitCommand(1))
            .andThen(cargo::runIntake, cargo)
            .andThen(
                ramsetePrototype
                    .copy()
                    .name("threeBallAuto1")
                    .field(null)
                    .traj(
                        TrajectoryGenerator.generateTrajectory(
                            pose(7.99, 2.81, -109.98),
                            List.of(),
                            pose(ballX, ballY, ballAngle),
                            trajConfig(drive)))
                    .build())
            .andThen(
                new NavXTurnToAngle<>(
                    turnAngle, 4, drive, pidAngleControllerPrototype.pid(0.001, 0, 0).build()))
            .andThen(
                ramsetePrototype
                    .copy()
                    .name("threeBallAuto2")
                    .field(null)
                    .traj(
                        TrajectoryGenerator.generateTrajectory(
                            pose(ballX, ballY, turnAngle),
                            List.of(),
                            pose(ballX2, ballY2, 140),
                            trajConfig(drive)))
                    .build())
            .andThen(
                new NavXTurnToAngle<>(turnAngle2, 4, drive, pidAngleControllerPrototype.build()))
            .andThen(
                ramsetePrototype
                    .name("threeBallAuto3")
                    .field(null)
                    .traj(
                        TrajectoryGenerator.generateTrajectory(
                            pose(ballX2, ballY2, turnAngle2),
                            List.of(),
                            hubPose,
                            trajConfig(drive).setReversed(true)))
                    .build())
            .andThen(cargo::spit, cargo)
            .andThen(new WaitCommand(2))
            .andThen(
                ramsetePrototype
                    .copy()
                    .name("threeBallAuto4")
                    .field(null)
                    .traj(
                        TrajectoryGenerator.generateTrajectory(
                            hubPose, List.of(), pose(5.70, 1.98, -90), trajConfig(drive)))
                    .build());
    var oneBallAuto =
        new InstantCommand(cargo::spit, cargo)
            .andThen(new WaitCommand(1))
            .andThen(cargo::stop, cargo)
            .andThen(new DriveAtSpeed<>(drive, 0.13, 5));
    var twoBallAuto =
        new InstantCommand(cargo::runIntake, cargo)
            .andThen(() -> drive.resetOdometry(pose(6.06, 5.13, 136.40)), drive)
            .andThen(cargo::deployIntake, cargo)
            .andThen(new WaitCommand(.4))
            .andThen(new DriveAtSpeed<>(drive, .13, 2))
            .andThen(new WaitCommand(.4))
            .andThen(new DriveAtSpeed<>(drive, -.13, 4.4))
            .andThen(cargo::spit, cargo)
            .andThen(new WaitCommand(2))
            .andThen(cargo::stop, cargo)
            .andThen(drive::fullStop, drive);

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

    // Auto
    Supplier<TrajectoryConfig> trajConfig =
        () ->
            new TrajectoryConfig(AUTO_MAX_SPEED, AUTO_MAX_ACCEL)
                .setKinematics(drive.getDriveKinematics())
                .addConstraint(
                    new DifferentialDriveVoltageConstraint(
                        driveFeedforward,
                        drive.getDriveKinematics(),
                        RobotController.getBatteryVoltage()));
    List<Command> autoStartupCommands =
        List.of(
            new InstantCommand(cargo::runIntake)
                .andThen(
                    ThreeBallAuto.createCommand(drive, cargo, ramsetePrototype, trajConfig, field))
                .andThen(spit.get()));

    List<Command> robotStartupCommands = List.of();

    List<Command> teleopStartupCommands =
        List.of(
            new InstantCommand(climber::disable),
            new InstantCommand(() -> drive.setDefaultCommand(driveDefaultCmd)),
            new InstantCommand(cargo::stop));

    List<Command> testStartupCommands = List.of();
    var allCommands =
        new CommandContainer(
            robotStartupCommands, autoStartupCommands, teleopStartupCommands, testStartupCommands);

    return new RobotMap(subsystems, pdp, allCommands, false);
  }

  @NotNull
  private static TrajectoryConfig trajConfig(@NotNull DriveUnidirectionalWithGyro drive) {
    return new TrajectoryConfig(AUTO_MAX_SPEED, AUTO_MAX_ACCEL)
        .setKinematics(drive.getDriveKinematics())
        .addConstraint(
            new DifferentialDriveVoltageConstraint(
                drive.getFeedforward(),
                drive.getDriveKinematics(),
                RobotController.getBatteryVoltage()))
        .setEndVelocity(0);
  }

  @NotNull
  private static Trajectory loadPathPlannerTraj(@NotNull String trajName) {
    var traj = PathPlanner.loadPath(trajName, AUTO_MAX_SPEED, AUTO_MAX_ACCEL, false);
    if (traj == null) {
      throw new Error("Trajectory not found: " + trajName);
    }
    return traj;
  }

  /** Create a command that immediately spits a ball, then goes to pick up a ball and comes back */
  private static Command oneThenOneBallTraj(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull RamseteBuilder ramsetePrototype,
      @NotNull Pose2d startPose,
      @NotNull Pose2d ballPose) {
    var toBall =
        TrajectoryGenerator.generateTrajectory(startPose, List.of(), ballPose, trajConfig(drive));
    var fromBall =
        TrajectoryGenerator.generateTrajectory(
            ballPose, List.of(), startPose, trajConfig(drive).setReversed(true));
    return new InstantCommand(cargo::spit, cargo)
        .andThen(new WaitCommand(1))
        .andThen(cargo::runIntake, cargo)
        .andThen(ramsetePrototype.copy().traj(toBall).build())
        .andThen(new WaitCommand(1))
        .andThen(ramsetePrototype.copy().traj(fromBall).build())
        .andThen(cargo::spit, cargo)
        .andThen(new WaitCommand(1))
        .andThen(cargo::stop, cargo);
  }

  /**
   * Create a command that picks up another ball, then goes home and spits both out
   *
   * @param endingPose Please ensure that the angle is reversed here!
   */
  private static Command twoBallTraj(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull RamseteBuilder ramsetePrototype,
      @NotNull Pose2d startPose,
      @NotNull Pose2d ballPose,
      @NotNull Pose2d endingPose) {
    var toBall =
        TrajectoryGenerator.generateTrajectory(startPose, List.of(), ballPose, trajConfig(drive));
    var fromBall =
        TrajectoryGenerator.generateTrajectory(
            ballPose, List.of(), endingPose, trajConfig(drive).setReversed(true));
    return new InstantCommand(cargo::runIntake, cargo)
        .andThen(new InstantCommand(cargo::deployIntake, cargo))
        .andThen(new WaitCommand(.4))
        .andThen(ramsetePrototype.copy().traj(toBall).build())
        .andThen(new WaitCommand(1))
        .andThen(ramsetePrototype.copy().traj(fromBall).build())
        .andThen(cargo::spit, cargo)
        .andThen(new WaitCommand(1))
        .andThen(cargo::stop, cargo);
  }

  /** Little helper because the verbosity of the Pose2d constructor is tiring */
  private static @NotNull Pose2d pose(double x, double y, double degrees) {
    return new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(degrees));
  }

  /** Add 180 to the heading */
  private static @NotNull Pose2d reverseHeading(Pose2d pose) {
    return pose(pose.getX(), pose.getY(), 180 + pose.getRotation().getDegrees());
  }
}
