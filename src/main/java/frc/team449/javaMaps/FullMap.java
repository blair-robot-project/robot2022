package frc.team449.javaMaps;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449._2022robot.cargo.Cargo2022;
import frc.team449._2022robot.climber.PivotingTelescopingClimber;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.UnidirectionalNavXDefaultDrive;
import frc.team449.drive.unidirectional.commands.motionprofiling.RamseteControllerCommands;
import frc.team449.generalInterfaces.doubleUnaryOperator.Polynomial;
import frc.team449.generalInterfaces.doubleUnaryOperator.RampComponent;
import frc.team449.javaMaps.builders.DriveSettingsBuilder;
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

public class FullMap {
  // Motor IDs
  public static final int RIGHT_LEADER_PORT = 1,
      RIGHT_LEADER_FOLLOWER_1_PORT = 11,
      RIGHT_LEADER_FOLLOWER_2_PORT = 7,
      LEFT_LEADER_PORT = 2,
      LEFT_LEADER_FOLLOWER_1_PORT = 4,
      LEFT_LEADER_FOLLOWER_2_PORT = 3,
      INTAKE_LEADER_PORT = 8,
      INTAKE_FOLLOWER_PORT = 9,
      SPITTER_PORT = 10,
      RIGHT_CLIMBER_MOTOR_PORT = 6,
      LEFT_CLIMBER_MOTOR_PORT = 5;

  // Controller ports
  public static final int MECHANISMS_JOYSTICK_PORT = 0, DRIVE_JOYSTICK_PORT = 1;
  // Button numbers
  public static final int INTAKE_NORMAL_BUTTON = 1, INTAKE_REVERSE_BUTTON = 3, SPIT_BUTTON = 2;
  // Speeds
  public static final double INTAKE_SPEED = 0.4, SPITTER_SPEED = 0.5;

  private FullMap() {}

  @NotNull
  public static RobotMap createRobotMap() {

    var pdp = new PDP(1, new RunningLinRegComponent(250, 0.75), PowerDistribution.ModuleType.kCTRE);

    var mechanismsJoystick = new RumbleableJoystick(MECHANISMS_JOYSTICK_PORT);
    var driveJoystick = new RumbleableJoystick(DRIVE_JOYSTICK_PORT);

    var navx = new AHRS(SerialPort.Port.kMXP, true);

    // Widget to show robot pose+trajectory in Glass
    var field = new Field2d();

    var driveMasterPrototype =
        new SparkMaxConfig()
            .setEnableBrakeMode(true)
            .setUnitPerRotation(0.31918) // 2 * Math.PI * 0.0508
            .setCurrentLimit(50)
            .setPostEncoderGearing(5.86)
            .setEnableVoltageComp(true);
    var rightMaster =
            driveMasterPrototype
                    .copy()
                    .setName("right")
                    .setPort(RIGHT_LEADER_PORT)
                    .setReverseOutput(false)
                    .addSlaveSpark(FollowerUtils.createFollowerSpark(RIGHT_LEADER_FOLLOWER_1_PORT), false)
                    .addSlaveSpark(FollowerUtils.createFollowerSpark(RIGHT_LEADER_FOLLOWER_2_PORT), false)
                    .createReal();
    var leftMaster =
            driveMasterPrototype
                    .copy()
                    .setPort(LEFT_LEADER_PORT)
                    .setName("left")
                    .setReverseOutput(true)
                    .addSlaveSpark(FollowerUtils.createFollowerSpark(LEFT_LEADER_FOLLOWER_1_PORT), false)
                    .addSlaveSpark(FollowerUtils.createFollowerSpark(LEFT_LEADER_FOLLOWER_2_PORT), false)
                    .createReal();

    var drive =
        new DriveUnidirectionalWithGyro(
            leftMaster,
            rightMaster,
            navx,
            new DriveSettingsBuilder()
                .leftFeedforward(new SimpleMotorFeedforward(0.20767, 2.2623, 0.1517))
                .rightFeedforward(new SimpleMotorFeedforward(0.20767, 2.2623, 0.1517))
                .build(),
            0.6492875);

    var throttlePrototype =
        new ThrottlePolynomialBuilder().stick(driveJoystick).smoothingTimeSecs(0.06);
    var rotThrottle =
        throttlePrototype
            .axis(0)
            .deadband(0.05)
            .inverted(true)
            .polynomial(
                new Polynomial(
                    Map.of(
                        1., 1.),
                        null))
            .build();
    var fwdThrottle =
        new ThrottleWithRamp(
            new ThrottleSum(
                Set.of(
                    throttlePrototype
                        .axis(3)
                        .deadband(0.05)
                        .inverted(true)
                        .polynomial(new Polynomial(Map.of(1., 1.), null))
                        .build(),
                    throttlePrototype.axis(2).inverted(false).build())),
            new RampComponent(1.5, .5));
    var oi =
        new OIArcadeWithDPad(
            rotThrottle,
            fwdThrottle,
            0.1,
            false,
            driveJoystick,
            new Polynomial(
                Map.of(
                    1., 1.), // Curvature
                null),
                .3,
            false);

      drive.setDefaultCommand(
              new UnidirectionalNavXDefaultDrive<>(
                      0,
                      new Debouncer(1.5),
                      0,
                      0.6,
                      null,
                      2,
                      3.0,
                      true,
                      .01,
                      0,
                      0.03,
                      new Debouncer(0.15),
                      drive,
                      oi,
                      null));

    var cargo =
        new Cargo2022(
            new SparkMaxConfig()
                .setName("intakeMotor")
                .setPort(INTAKE_LEADER_PORT)
                .addSlaveSpark(FollowerUtils.createFollowerSpark(INTAKE_FOLLOWER_PORT), true)
                .createReal(),
            new SparkMaxConfig().setName("spitterMotor").setPort(SPITTER_PORT).setEnableBrakeMode(false).createReal(),
            INTAKE_SPEED,
            SPITTER_SPEED);

    var climber =
        new PivotingTelescopingClimber(
            driveMasterPrototype
                .copy()
                .setName("climber_right")
                .setPort(RIGHT_CLIMBER_MOTOR_PORT)
                .setEnableBrakeMode(true)
                .setUnitPerRotation(0.1949)
                .setPostEncoderGearing(10)
                .setReverseOutput(false)
                .createReal(),
            driveMasterPrototype
                .copy()
                .setName("climber_left")
                .setPort(LEFT_CLIMBER_MOTOR_PORT)
                .setEnableBrakeMode(true)
                .setUnitPerRotation(0.1949)
                .setPostEncoderGearing(10)
                .setReverseOutput(true)
                .createReal(),
            12,
            0,
            0,
            .5, // m/s
            .5, // m/s^2
            .3 // m
            );

    // PUT YOUR SUBSYSTEM IN HERE AFTER INITIALIZING IT
    var subsystems = List.<Subsystem>of(drive, cargo, climber);

    var updater =
        new Updater(List.of(pdp, navx, oi, () -> field.setRobotPose(drive.getCurrentPose())));

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
        .whenPressed(
            () -> {
              climber.disable();
              climber.leftArm.set(0.1);
              climber.rightArm.set(0.1);
            })
        .whenReleased(
            () -> {
              climber.enable();
              climber.resetController();
              climber.setGoal(climber.getMeasurement());
            });

    new JoystickButton(mechanismsJoystick, XboxController.Button.kB.value)
        .whenPressed(
            () -> {
              climber.disable();
              climber.leftArm.set(-0.2);
              climber.rightArm.set(-0.2);
            })
        .whenReleased(
            () -> {
              climber.enable();
              climber.resetController();
              climber.setGoal(climber.getMeasurement());
            });
    // new JoystickButton(mechanismsJoystick, XboxController.Button.kY.value)
    //  .whenPressed(new ExtendTelescopingArm(climber));
    //        .whenPressed(
    //            () -> {
    //              climber.disable();
    //              climber.leftArm.set(0.1);
    //            });
    // new JoystickButton(mechanismsJoystick, XboxController.Button.kX.value)
    //                .whenPressed(
    //                    () -> {
    //                      climber.disable();
    //                      climber.leftArm.set(-0.1);
    //                    });
    // .whenPressed(new RetractTelescopingArm(climber));
    //        new JoystickButton(mechanismsJoystick, XboxController.Button.kX.value)
    //            .whenPressed(climber::pivotTelescopingArmIn, climber);
    //        new JoystickButton(mechanismsJoystick, XboxController.Button.kB.value)
    //            .whenPressed(climber::pivotTelescopingArmOut, climber);

    var ramsete =
        RamseteControllerCommands.goToPosition(
            drive,
            .009,
            .009,
            .001,
            new PIDController(.001, 0, 0),
            new PIDController(.001, 0, 0),
            new Pose2d(new Translation2d(0.0, 1), new Rotation2d(0.0)),
            List.of(),
            false,
            field);

    List<Command> robotStartupCommands = List.of();

    List<Command> autoStartupCommands = List.of(
            new InstantCommand(drive::resetPosition),
            ramsete);

    List<Command> teleopStartupCommands =
        List.of(
            new InstantCommand(climber::reset));

    List<Command> testStartupCommands = List.of();
    var allCommands =
        new CommandContainer(
            robotStartupCommands, autoStartupCommands, teleopStartupCommands, testStartupCommands);

    return new RobotMap(subsystems, pdp, updater, allCommands, false);
  }
}
