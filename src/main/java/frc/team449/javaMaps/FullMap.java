package frc.team449.javaMaps;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.team449._2022robot.cargo.Cargo2022;
import frc.team449._2022robot.climber.ClimberArm;
import frc.team449._2022robot.climber.PivotingTelescopingClimber;
import frc.team449.ahrs.AHRS;
import frc.team449.ahrs.PIDAngleControllerBuilder;
import frc.team449.auto.routines.ThreeBallAuto;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.drive.DriveSettingsBuilder;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.UnidirectionalNavXDefaultDrive;
import frc.team449.generalInterfaces.doubleUnaryOperator.Polynomial;
import frc.team449.generalInterfaces.doubleUnaryOperator.RampComponent;
import frc.team449.generalInterfaces.limelight.Limelight;
import frc.team449.motor.builder.SparkMaxConfig;
import frc.team449.oi.throttles.ThrottlePolynomialBuilder;
import frc.team449.oi.throttles.ThrottleSum;
import frc.team449.oi.throttles.ThrottleWithRamp;
import frc.team449.oi.unidirectional.arcade.OIArcadeWithDPad;
import frc.team449.other.Debouncer;
import frc.team449.other.FollowerUtils;
import frc.team449.other.Updater;
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
      RIGHT_EXTERNAL_FWD_PORT = 8,
      RIGHT_EXTERNAL_REV_PORT = 9;

  // Other CAN IDs
  public static final int PDP_CAN = 1, PCM_MODULE = 0;
  // Controller ports
  public static final int CARGO_JOYSTICK_PORT = 0,
      DRIVE_JOYSTICK_PORT = 1,
      CLIMBER_JOYSTICK_PORT = 2;
  // Limelight
  public static final int DRIVER_PIPELINE = 0; // TODO find out what this is!
  // Speeds
  public static final double INTAKE_SPEED = 0.8, SPITTER_SPEED = 0.6;
  public static final double AUTO_MAX_SPEED = 1.9, AUTO_MAX_ACCEL = .2;
  // Drive constants
  public static final double DRIVE_WHEEL_RADIUS = Units.inchesToMeters(2);
  public static final double DRIVE_GEARING = 1; // 5.86;
  public static final int DRIVE_ENCODER_CPR = 256;
  public static final int DRIVE_CURRENT_LIM = 40;
  public static final double DRIVE_KP_VEL = 3.201, // 27.2,
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
      DRIVE_ANGLE_FF_KA = 22.755;
  // old value from measuring from the outside of the wheel: 0.6492875
  // measuring from the inside of the wheel : .57785
  public static final double DRIVE_TRACK_WIDTH = 0.61401; // 0.6492875;
  // todo find these using sysid
  public static final double MOMENT_OF_INERTIA = 7.5;
  public static final double MASS = 60;
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

  // Ramping
  public static final double RAMP_INCREASE = 0.9, RAMP_DECREASE = 0.7;
  public static final int BLUE_PIPELINE = 1, RED_PIPELINE = 2;

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

    var driveMasterPrototype =
        new SparkMaxConfig()
            .setEnableBrakeMode(true)
            .setUnitPerRotation(
                0.3021211527151539) // 2 * Math.PI * DRIVE_WHEEL_RADIUS) // = 0.3191858136
            .setCurrentLimit(DRIVE_CURRENT_LIM)
            .setPostEncoderGearing(DRIVE_GEARING)
            .setEncoderCPR(DRIVE_ENCODER_CPR)
            .setEnableVoltageComp(true);

    // todo use sysid gains to make this
    var driveSim =
        new DifferentialDrivetrainSim(
            LinearSystemId.identifyDrivetrainSystem(
                DRIVE_FF_KV, DRIVE_FF_KA, DRIVE_ANGLE_FF_KV, DRIVE_ANGLE_FF_KA, DRIVE_TRACK_WIDTH),
            DCMotor.getNEO(3),
            DRIVE_GEARING,
            DRIVE_TRACK_WIDTH,
            DRIVE_WHEEL_RADIUS,
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    var leftExtEnc = new Encoder(LEFT_EXTERNAL_FWD_PORT, LEFT_EXTERNAL_REV_PORT);
    var rightExtEnc = new Encoder(RIGHT_EXTERNAL_FWD_PORT, RIGHT_EXTERNAL_REV_PORT);
    rightExtEnc.setReverseDirection(true);
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

    var drive =
        DriveUnidirectionalWithGyro.createRealOrSim(
            leftMaster,
            rightMaster,
            navx,
            new DriveSettingsBuilder()
                .feedforward(new SimpleMotorFeedforward(DRIVE_FF_KS, DRIVE_FF_KV, DRIVE_FF_KA))
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
            .pid(221.18, 0, 0.03);

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
        driveMasterPrototype
            .copy()
            .setCurrentLimit(null)
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

    Supplier<Command> spit =
        () ->
            new InstantCommand(cargo::spit, cargo)
                .andThen(new WaitCommand(1))
                .andThen(cargo::stop, cargo);

    List<Command> autoStartupCommands =
        List.of(
            new InstantCommand(cargo::runIntake)
                .andThen(
                    ThreeBallAuto.createCommand(
                        drive,
                        cargo,
                        DRIVE_KP_VEL,
                        DRIVE_KD_VEL,
                        new SimpleMotorFeedforward(DRIVE_FF_KS, DRIVE_FF_KV, DRIVE_FF_KA),
                        new SimpleMotorFeedforward(DRIVE_FF_KS, DRIVE_FF_KV, DRIVE_FF_KA),
                        field))
                .andThen(spit.get()));

    List<Command> robotStartupCommands = List.of();

    List<Command> teleopStartupCommands =
        List.of(
            new InstantCommand(climber::disable),
            new InstantCommand(() -> drive.setDefaultCommand(driveDefaultCmd)),
            new InstantCommand(cargo::stop));
    //            new IntakeLimelightCommand(cargo, limelight, BLUE_PIPELINE, RED_PIPELINE)
    //                .perpetually());

    List<Command> testStartupCommands = List.of();
    var allCommands =
        new CommandContainer(
            robotStartupCommands, autoStartupCommands, teleopStartupCommands, testStartupCommands);

    return new RobotMap(subsystems, pdp, allCommands, false);
  }
}
// Hi there!
