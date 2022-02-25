package frc.team449.javaMaps;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449._2022robot.cargo.Cargo2022;
import frc.team449._2022robot.climber.ClimberArm;
import frc.team449._2022robot.climber.PivotingTelescopingClimber;
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

import static frc.team449.javaMaps.FullMap.*;

public final class SimMap {
  public static final double MOMENT_OF_INERTIA = 1;
  public static final double MASS = 1;

  private SimMap() {}

  @NotNull
  public static RobotMap createRobotMap() {
    var pdp =
        new PDP(PDP_CAN, new RunningLinRegComponent(250, 0.75), PowerDistribution.ModuleType.kCTRE);
    var mechanismsJoystick = new RumbleableJoystick(MECHANISMS_JOYSTICK_PORT);
    var driveJoystick = new RumbleableJoystick(DRIVE_JOYSTICK_PORT);

    var ahrs = new AHRS(SerialPort.Port.kMXP, true);

    // Widget to show robot pose+trajectory in Glass
    var field = new Field2d();
    SmartDashboard.putData(field);

    var limelight = new Limelight(DRIVER_PIPELINE);

    var driveMasterPrototype =
        new SparkMaxConfig()
            .setEnableBrakeMode(true)
            .setUnitPerRotation(2 * Math.PI * DRIVE_WHEEL_RADIUS)
            .setCurrentLimit(DRIVE_CURRENT_LIM)
            .setPostEncoderGearing(DRIVE_GEARING)
            .setEnableVoltageComp(true);
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

    //todo use sysid gains to make this
    var driveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getNeo550(3),
            DRIVE_GEARING,
            MOMENT_OF_INERTIA,
            MASS,
            DRIVE_WHEEL_RADIUS,
            DRIVE_TRACK_WIDTH,
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    var drive =
        new DriveUnidirectionalWithGyro(
            leftMaster,
            rightMaster,
            ahrs,
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
            new RampComponent(.7, .50));
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
            0,
            new Debouncer(1.5),
            0,
            0.6,
            null,
            2,
            3.0,
            false,
            .01,
            0,
            0.03,
            new Debouncer(0.15),
            drive,
            oi,
            null);

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
    var stopCargo = new InstantCommand(cargo::stop, cargo);

    var armPrototype =
        driveMasterPrototype
            .copy()
            .setRevSoftLimit(0.)
            .setFwdSoftLimit(CLIMBER_DISTANCE)
            .setUnitPerRotation(0.1949)
            .setEnableBrakeMode(true);
    var leftArm =
        new ClimberArm(
            armPrototype
                .copy()
                .setName("climber_left")
                .setPort(LEFT_CLIMBER_MOTOR_PORT)
                .setPostEncoderGearing(10)
                .setReverseOutput(true)
                .createReal(),
            new PIDController(12, 0, 0),
            new ElevatorFeedforward(0, 0, 0));
    var rightArm =
        new ClimberArm(
            driveMasterPrototype
                .copy()
                .setName("climber_right")
                .setPort(RIGHT_CLIMBER_MOTOR_PORT)
                .setPostEncoderGearing(10)
                .setReverseOutput(false)
                .createReal(),
            new PIDController(12, 0, 0),
            new ElevatorFeedforward(0, 0, 0));
    var climber = new PivotingTelescopingClimber(leftArm, rightArm, CLIMBER_DISTANCE);

    // PUT YOUR SUBSYSTEM IN HERE AFTER INITIALIZING IT
    var subsystems = List.<Subsystem>of(drive, cargo, climber);

    var updater =
        new Updater(List.of(pdp, ahrs, oi, () -> field.setRobotPose(drive.getCurrentPose())));

    // Button bindings here
    // Take in balls but don't shoot
    //    new SimpleButton(mechanismsJoystick, XboxController.Button.kA.value)
    //        .whileHeld(cargo::runIntake, cargo)
    //        .whenReleased(cargo::stop, cargo);
    // Run all motors in intake to spit balls out
    //    new SimpleButton(mechanismsJoystick, XboxController.Button.kB.value)
    //        .whileHeld(cargo::spit, cargo)
    //        .whenReleased(cargo::stop, cargo);

    // TODO BUTTON BINDINGS HERE
    new JoystickButton(mechanismsJoystick, XboxController.Button.kA.value)
        .whileActiveContinuous(
            new WaitCommand(0.01)
                .andThen(() -> climber.setSetpoint(climber.getSetpoint() + 0.01), climber));
    new JoystickButton(mechanismsJoystick, XboxController.Button.kB.value)
        .whileActiveContinuous(
            new WaitCommand(0.01)
                .andThen(() -> climber.setSetpoint(climber.getSetpoint() - 0.01), climber));

    var ramsetePrototype =
        new RamseteBuilder()
            .drivetrain(drive)
            .leftPidController(new PIDController(DRIVE_KP_VEL, 0, DRIVE_KD_VEL))
            .rightPidController(new PIDController(DRIVE_KP_VEL, 0, DRIVE_KD_VEL))
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
            new InstantCommand(climber::enable),
            new InstantCommand(() -> drive.setDefaultCommand(driveDefaultCmd)),
            new InstantCommand(cargo::stop));

    List<Command> testStartupCommands = List.of();
    var allCommands =
        new CommandContainer(
            robotStartupCommands, autoStartupCommands, teleopStartupCommands, testStartupCommands);

    return new RobotMap(subsystems, pdp, updater, allCommands, false);
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
