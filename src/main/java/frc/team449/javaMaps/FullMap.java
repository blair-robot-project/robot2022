package frc.team449.javaMaps;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.team449.drive.unidirectional.commands.DriveAtSpeed;
import frc.team449.drive.unidirectional.commands.NavXTurnToAngle;
import frc.team449.drive.unidirectional.commands.UnidirectionalNavXDefaultDrive;
import frc.team449.motor.builder.SparkMaxConfig;
import frc.team449.oi.RampComponent;
import frc.team449.oi.joystick.RumbleCommand;
import frc.team449.oi.throttles.Polynomial;
import frc.team449.oi.throttles.ThrottlePolynomialBuilder;
import frc.team449.oi.throttles.ThrottleSum;
import frc.team449.oi.throttles.ThrottleWithRamp;
import frc.team449.oi.unidirectional.arcade.OIArcadeWithDPad;
import frc.team449.other.FollowerUtils;
import frc.team449.robot2022.cargo.Cargo2022;
import frc.team449.robot2022.cargo.IntakeLimelightRumbleComponent;
import frc.team449.robot2022.climber.ClimberArm;
import frc.team449.robot2022.climber.ClimberLimitRumbleComponent;
import frc.team449.robot2022.climber.PivotingTelescopingClimber;
import frc.team449.robot2022.routines.BadStationTwoBallAuto;
import frc.team449.updatable.Updater;
import frc.team449.wrappers.Limelight;
import frc.team449.wrappers.PDP;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import static frc.team449.robot2022.DriveConstants.*;
import static frc.team449.robot2022.cargo.CargoConstants.*;
import static frc.team449.robot2022.climber.ClimberConstants.*;

public class FullMap {
  // Other CAN IDs
  public static final int PDP_CAN = 1, PCM_MODULE = 0;
  // Controller ports
  public static final int CARGO_JOYSTICK_PORT = 0,
      DRIVE_JOYSTICK_PORT = 1,
      CLIMBER_JOYSTICK_PORT = 2;
  // Limelight
  public static final int DRIVER_PIPELINE = 0, BLUE_PIPELINE = 1, RED_PIPELINE = 2;
  // Speeds
  public static final double AUTO_MAX_SPEED = 2, AUTO_MAX_ACCEL = .4;

  private FullMap() {}

  @NotNull
  public static RobotMap createRobotMap() {
    var pdp =
        new PDP(PDP_CAN, new RunningLinRegComponent(250, 0.75), PowerDistribution.ModuleType.kCTRE);
    var cargoJoystick = new Joystick(CARGO_JOYSTICK_PORT);
    var driveJoystick = new Joystick(DRIVE_JOYSTICK_PORT);
    var climberJoystick = new Joystick(CLIMBER_JOYSTICK_PORT);

    var navx = AHRS.createRealOrSim(SerialPort.Port.kMXP, true);

    // Widget to show robot pose+trajectory in Glass
    var field = new Field2d();
    SmartDashboard.putData(field);

    var limelight = new Limelight(DRIVER_PIPELINE);
    limelight.setStreamMode(Limelight.StreamMode.STANDARD);
    limelight.setLedMode(Limelight.LedMode.OFF);

    var intakeLimelightRumbleCommand =
        new RumbleCommand(
            List.of(cargoJoystick),
            new IntakeLimelightRumbleComponent(limelight, BLUE_PIPELINE, RED_PIPELINE));

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
                .leftVelPID(new PIDController(DRIVE_KP_VEL, DRIVE_KI_VEL, DRIVE_KD_VEL))
                .rightVelPID(new PIDController(DRIVE_KP_VEL, DRIVE_KI_VEL, DRIVE_KD_VEL))
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
        new UnidirectionalNavXDefaultDrive(
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
            CLIMBER_DIFFERENTIATION_HEIGHT,
            CLIMBER_MID_DISTANCE,
            RobotBase.isReal() && CLIMBER_HAS_SENSORS
                ? new DigitalInput(CLIMBER_LEFT_SENSOR_CHANNEL)::get
                : () -> false);
    var rightArm =
        new ClimberArm(
            armPrototype
                .copy()
                .setName("climber_right")
                .setPort(RIGHT_CLIMBER_MOTOR_PORT)
                .setUnitPerRotation(CLIMBER_RIGHT_UPR)
                .setReverseOutput(false)
                .createReal(),
            CLIMBER_DIFFERENTIATION_HEIGHT,
            CLIMBER_MID_DISTANCE,
            RobotBase.isReal() && CLIMBER_HAS_SENSORS
                ? new DigitalInput(CLIMBER_RIGHT_SENSOR_CHANNEL)::get
                : () -> false);
    var pivotPiston =
        new DoubleSolenoid(
            PCM_MODULE,
            PneumaticsModuleType.CTREPCM,
            CLIMBER_PISTON_FWD_CHANNEL,
            CLIMBER_PISTON_REV_CHANNEL);
    var climber =
        new PivotingTelescopingClimber(
            leftArm, rightArm, pivotPiston, CLIMBER_DISTANCE, CLIMBER_MID_DISTANCE);

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

    // Rumbles the joystick when either arm gets within `CLIMBER_RUMBLE_THRESHOLD` of the limits.
    // Rumbling increases linearly.
    var climberRumbleCommand =
        new RumbleCommand(
            List.of(climberJoystick),
            new ClimberLimitRumbleComponent(climber, CLIMBER_RUMBLE_TOLERANCE));

    var ramsetePrototype =
        new RamseteBuilder()
            .drivetrain(drive)
            .anglePID(
                new PIDAngleControllerBuilder()
                    .absoluteTolerance(0.001)
                    .onTargetBuffer(null)
                    .minimumOutput(0)
                    .maximumOutput(null)
                    .loopTimeMillis(20)
                    .deadband(2)
                    .inverted(false)
                    .pid(.006, 0, 0.03)
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
                new NavXTurnToAngle(
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
            .andThen(new NavXTurnToAngle(turnAngle2, 4, drive, pidAngleControllerPrototype.build()))
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
            .andThen(new DriveAtSpeed(drive, 0.13, 5));
    var twoBallAuto =
        new InstantCommand(cargo::runIntake, cargo)
            .andThen(() -> drive.resetOdometry(pose(6.06, 5.13, 136.40)), drive)
            .andThen(cargo::deployIntake, cargo)
            .andThen(new WaitCommand(.4))
            .andThen(new DriveAtSpeed(drive, .13, 2))
            .andThen(new WaitCommand(.4))
            .andThen(new DriveAtSpeed(drive, -.13, 4.4))
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
                    BadStationTwoBallAuto.createCommand(
                        drive, cargo, ramsetePrototype, trajConfig, field))
                .andThen(spit.get()));

    List<Command> robotStartupCommands = List.of();

    List<Command> teleopStartupCommands =
        List.of(
            new InstantCommand(() -> drive.setDefaultCommand(driveDefaultCmd)),
            new InstantCommand(cargo::stop),
            climberRumbleCommand,
            intakeLimelightRumbleCommand);

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
