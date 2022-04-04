package frc.team449.robot2022;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449.ahrs.AHRS;
import frc.team449.ahrs.PIDAngleControllerBuilder;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.drive.DriveSettingsBuilder;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.UnidirectionalNavXDefaultDrive;
import frc.team449.motor.builder.SparkMaxConfig;
import frc.team449.multiSubsystem.StateSpaceModelBuilder;
import frc.team449.oi.RampComponent;
import frc.team449.oi.joystick.RumbleCommand;
import frc.team449.oi.throttles.Polynomial;
import frc.team449.oi.throttles.ThrottlePolynomialBuilder;
import frc.team449.oi.throttles.ThrottleSum;
import frc.team449.oi.throttles.ThrottleWithRamp;
import frc.team449.oi.unidirectional.arcade.OIArcadeWithDPad;
import frc.team449.other.FollowerUtils;
import frc.team449.robot2022.cargo.Cargo2022;
import frc.team449.robot2022.climber.ClimberArm;
import frc.team449.robot2022.climber.ClimberLimitRumbleComponent;
import frc.team449.robot2022.climber.PivotingTelescopingClimber;
import frc.team449.robot2022.routines.AutoConstants;
import frc.team449.robot2022.routines.ThreeBallHighCurvyAuto;
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
  /** Control loop time */
  public static final double LOOP_TIME = 0.02;

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

    var driveMasterPrototype =
        new SparkMaxConfig()
            .setEnableBrakeMode(true)
            .setUnitPerRotation(DRIVE_UPR)
            .setCurrentLimit(DRIVE_CURRENT_LIM)
            .setPostEncoderGearing(DRIVE_GEARING)
            .setEncoderCPR(NEO_ENCODER_CPR)
            .setExtEncoderCPR(DRIVE_EXT_ENCODER_CPR)
            //            .setUseInternalEncAsFallback(DRIVE_ENC_POS_THRESHOLD,
            // DRIVE_ENC_VEL_THRESHOLD)
            .setEnableVoltageComp(true);

    var drivePlant =
        LinearSystemId.identifyDrivetrainSystem(
            DRIVE_FF_KV, DRIVE_FF_KA, DRIVE_ANGLE_FF_KV, DRIVE_ANGLE_FF_KA);
    var driveSim =
        new DifferentialDrivetrainSim(
            drivePlant,
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

    // todo characterize and then actually use this
    var driveLoop =
        new StateSpaceModelBuilder<>(Nat.N2())
            .loopTime(LOOP_TIME)
            .plant(drivePlant)
            .stateStdDevs(VecBuilder.fill(3.0, 3.0))
            .measStdDevs(VecBuilder.fill(0.01, 0.01))
            .errorTolerances(VecBuilder.fill(1.0, 1.0))
            .maxControlEfforts(VecBuilder.fill(12, 12))
            .build();

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

    var flywheelLoop =
        new StateSpaceModelBuilder<>(Nat.N1())
            .loopTime(LOOP_TIME)
            .plant(LinearSystemId.identifyVelocitySystem(SHOOTER_KV, SHOOTER_KA))
            .stateStdDevs(VecBuilder.fill(3.0))
            .measStdDevs(VecBuilder.fill(0.01))
            .errorTolerances(VecBuilder.fill(SHOOTER_TOLERANCE))
            .maxControlEfforts(VecBuilder.fill(RobotController.getBatteryVoltage()))
            .build();

    var cargo =
        new Cargo2022(
            new SparkMaxConfig()
                .setName("intakeMotor")
                .setPort(INTAKE_LEADER_PORT)
                .setUnitPerRotation(1)
                .setCurrentLimit(INTAKE_CURR_LIM)
                .setEncoderCPR(NEO_ENCODER_CPR)
                .addSlaveSpark(FollowerUtils.createFollowerSpark(INTAKE_FOLLOWER_PORT), true)
                .createReal(),
            new SparkMaxConfig()
                .setName("spitterMotor")
                .setPort(SPITTER_PORT)
                .setEnableBrakeMode(false)
                .setEncoderCPR(NEO_ENCODER_CPR)
                .createReal(),
            flywheelLoop,
            new SimpleMotorFeedforward(SPITTER_KS, SPITTER_KV, SPITTER_KA),
            new SparkMaxConfig()
                .setName("flywheelMotor")
                .setPort(FLYWHEEL_MOTOR_PORT)
                .setEncoderCPR(NEO_ENCODER_CPR)
                .setEnableBrakeMode(false)
                .setCalculateVel(true)
                .createReal(),
            new SimpleMotorFeedforward(SHOOTER_KS, SHOOTER_KV, SHOOTER_KA),
            new DoubleSolenoid(
                PCM_MODULE,
                PneumaticsModuleType.CTREPCM,
                INTAKE_PISTON_FWD_CHANNEL,
                INTAKE_PISTON_REV_CHANNEL),
            new DoubleSolenoid(
                PCM_MODULE,
                PneumaticsModuleType.CTREPCM,
                HOOD_PISTON_FWD_CHANNEL,
                HOOD_PISTON_REV_CHANNEL));

    var armPrototype =
        new SparkMaxConfig()
            .setEnableVoltageComp(true)
            //            .setRevSoftLimit(0.)
            //            .setFwdSoftLimit(CLIMBER_DISTANCE)
            .setPostEncoderGearing(CLIMBER_GEARING)
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
            CLIMBER_DISTANCE,
            RobotBase.isReal() && CLIMBER_HAS_SENSORS
                ? new DigitalInput(CLIMBER_LEFT_SENSOR_CHANNEL)::get
                : () -> true);
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
            CLIMBER_DISTANCE,
            RobotBase.isReal() && CLIMBER_HAS_SENSORS
                ? new DigitalInput(CLIMBER_RIGHT_SENSOR_CHANNEL)::get
                : () -> true);
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
            CLIMBER_DISTANCE,
            CLIMBER_MID_DISTANCE,
            CLIMBER_EXTEND_VEL,
            CLIMBER_RETRACT_VEL,
            CLIMBER_RETRACT_VEL_SLOW);

    // PUT YOUR SUBSYSTEM IN HERE AFTER INITIALIZING IT
    var subsystems = List.of(drive, cargo, climber);

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
        .whenHeld(cargo.startShooterCommand())
        .whenReleased(cargo::stop);
    // Stop the flywheel for shooting
    new JoystickButton(cargoJoystick, XboxController.Button.kY.value).whenHeld(cargo.ready());
    // Stow/retract intake
    new JoystickButton(cargoJoystick, XboxController.Button.kX.value)
        .whenPressed(cargo::retractIntake);
    // Deploy intake
    new JoystickButton(cargoJoystick, XboxController.Button.kA.value)
        .whileHeld(cargo::deployIntake, cargo)
        .whenReleased(cargo::stop, cargo);
    // Driver joystick intake deploy and retract controls
    // Stow/retract intake
    new JoystickButton(driveJoystick, XboxController.Button.kX.value)
        .whenPressed(cargo::retractIntake);
    // Deploy intake
    new JoystickButton(driveJoystick, XboxController.Button.kA.value)
        .whileHeld(cargo::deployIntake, cargo)
        .whenReleased(cargo::stop, cargo);
    // Run intake backwards
    new JoystickButton(cargoJoystick, XboxController.Button.kB.value)
        .whileHeld(cargo::runIntakeReverse, cargo)
        .whenReleased(cargo::stop, cargo);
    // Deploy hood
    new POVButton(cargoJoystick, 0).whenPressed(cargo::deployHood, cargo);
    // Remove hood
    new POVButton(cargoJoystick, 180).whenPressed(cargo::removeHood, cargo);

    // Extend Climber override
    new JoystickButton(climberJoystick, XboxController.Button.kY.value)
        .whileHeld(() -> climber.setRaw(CLIMBER_EXTEND_VEL), climber)
        .whenReleased(() -> climber.setRaw(0), climber);
    // Retract climber override
    new JoystickButton(climberJoystick, XboxController.Button.kA.value)
        .whileHeld(() -> climber.setRaw(CLIMBER_RETRACT_VEL), climber)
        .whenReleased(() -> climber.setRaw(0), climber);
    // Extend Climber
    new POVButton(climberJoystick, 0)
        .whileHeld(climber::setExtend, climber)
        .whenReleased(climber::stop, climber);
    // Retract climber
    new POVButton(climberJoystick, 180)
        .whileHeld(climber::setRetract, climber)
        .whenReleased(climber::stop, climber);
    // Pivot climber out
    new JoystickButton(climberJoystick, XboxController.Button.kX.value)
        .whenPressed(
            new InstantCommand(cargo::deployIntake).andThen(climber::pivotTelescopingArmOut));
    // Retract climber arm in with piston
    new JoystickButton(climberJoystick, XboxController.Button.kB.value)
        .whenPressed(climber::pivotTelescopingArmIn);

    // Rumbles the joystick when either arm gets within `CLIMBER_RUMBLE_THRESHOLD` of the limits.
    // Rumbling increases linearly.
    var climberRumbleCommand =
        new RumbleCommand(
            new ClimberLimitRumbleComponent(climber, CLIMBER_RUMBLE_TOLERANCE), climberJoystick);

    // Auto
    Supplier<TrajectoryConfig> trajConfig =
        () ->
            new TrajectoryConfig(AutoConstants.AUTO_MAX_SPEED, AutoConstants.AUTO_MAX_ACCEL)
                .setKinematics(drive.getDriveKinematics())
                .addConstraint(
                    new DifferentialDriveVoltageConstraint(
                        driveFeedforward,
                        drive.getDriveKinematics(),
                        RobotController.getBatteryVoltage()))
        //                .addConstraint(
        //                    new CentripetalAccelerationConstraint(
        //                        AutoConstants.AUTO_MAX_CENTRIPETAL_ACCEL)
        //                )
        ;
    List<Command> autoStartupCommands =
        List.of(
            ThreeBallHighCurvyAuto.createCommand(drive, cargo, trajConfig, field)
                .andThen(new WaitCommand(AutoConstants.PAUSE_AFTER_SPIT))
                .andThen(cargo::stop, cargo));

    List<Command> robotStartupCommands = List.of();

    List<Command> teleopStartupCommands =
        List.of(
            new InstantCommand(() -> drive.setDefaultCommand(driveDefaultCmd)),
            new InstantCommand(climber::pivotTelescopingArmIn, climber),
            new InstantCommand(cargo::stop, cargo),
            new InstantCommand(cargo::deployHood) /*,
            //            climberRumbleCommand,
            intakeLimelightRumbleCommand*/);

    List<Command> testStartupCommands = List.of();
    var allCommands =
        new CommandContainer(
            robotStartupCommands, autoStartupCommands, teleopStartupCommands, testStartupCommands);

    var otherLoggables = List.of(driveDefaultCmd);

    return new RobotMap(subsystems, pdp, allCommands, otherLoggables, false);
  }
}
