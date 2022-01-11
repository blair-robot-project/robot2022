package frc.team449.javaMaps;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team449.CommandContainer;
import frc.team449.RobotMap;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.drive.unidirectional.commands.DriveAtSpeed;
import frc.team449.drive.unidirectional.commands.UnidirectionalNavXDefaultDrive;
import frc.team449.generalInterfaces.doubleUnaryOperator.Polynomial;
import frc.team449.generalInterfaces.doubleUnaryOperator.RampComponent;
import frc.team449.jacksonWrappers.*;
import frc.team449.javaMaps.builders.DriveSettingsBuilder;
import frc.team449.javaMaps.builders.SparkMaxConfig;
import frc.team449.javaMaps.builders.ThrottlePolynomialBuilder;
import frc.team449.oi.buttons.CommandButton;
import frc.team449.oi.buttons.SimpleButton;
import frc.team449.oi.throttles.Throttle;
import frc.team449.oi.throttles.ThrottleSum;
import frc.team449.oi.unidirectional.arcade.OIArcadeWithDPad;
import frc.team449.other.Debouncer;
import frc.team449.other.DefaultCommand;
import frc.team449.other.Updater;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Map;

public class FullMap {
  // Motor IDs
  public static final int RIGHT_LEADER_PORT = 2,
      RIGHT_LEADER_FOLLOWER_1_PORT = 3,
      LEFT_LEADER_PORT = 1,
      LEFT_LEADER_FOLLOWER_1_PORT = 4,
      ELEVATOR_MOTOR_PORT = 5;
  // Controller ports
  public static final int MECHANISMS_JOYSTICK_PORT = 0, DRIVE_JOYSTICK_PORT = 1;
  // Motor speeds
  public static final double ELEVATOR_MAX_VELOCITY = .02, ELEVATOR_MAX_ACCEL = 0.007;
  // Mechs button numbers
  public static final int ELEVATOR_MOVE_TO_TOP = 1,
      ELEVATOR_MOVE_TO_UPPER = 2,
      ELEVATOR_MOVE_TO_LOWER = 3,
      ELEVATOR_MOVE_TO_BOTTOM = 4,
      ELEVATOR_MOVE_UP = 1,
      ELEVATOR_MOVE_DOWN = 3,
      INTAKE_OPEN_BUTTON = 7,
      INTAKE_CLOSE_BUTTON = 8;
  // Solenoid ports
  private static final int INTAKE_SOLENOID_FORWARD_CHANNEL = 2, INTAKE_SOLENOID_REVERSE_CHANNEL = 3;

  private FullMap() {}

  @NotNull
  public static RobotMap createRobotMap() {
    var pdp = new PDP(1, new RunningLinRegComponent(250, 0.75));

    var mechanismsJoystick = new MappedJoystick(MECHANISMS_JOYSTICK_PORT);
    var driveJoystick = new MappedJoystick(DRIVE_JOYSTICK_PORT);
    var joysticks = List.of(mechanismsJoystick, driveJoystick);

    var intake =
        new DoubleSolenoid(
            0,
            PneumaticsModuleType.CTREPCM,
            INTAKE_SOLENOID_FORWARD_CHANNEL,
            INTAKE_SOLENOID_REVERSE_CHANNEL);

    var navx = new MappedAHRS(SerialPort.Port.kMXP, true);
    var driveMasterPrototype =
        new SparkMaxConfig()
            .setEnableBrakeMode(true)
            .setUnitPerRotation(0.4787787204060999)
            .setCurrentLimit(50)
            .setEnableVoltageComp(true);
    var rightMaster =
        WrappedMotor.createSpark(
            driveMasterPrototype
                .copy()
                .setName("right")
                .setPort(RIGHT_LEADER_PORT)
                .setReverseOutput(false)
                .setSlaveSparks(List.of(new SlaveSparkMax(RIGHT_LEADER_FOLLOWER_1_PORT, false))));
    var leftMaster =
        WrappedMotor.createSpark(
            driveMasterPrototype
                .copy()
                .setPort(LEFT_LEADER_PORT)
                .setName("left")
                .setReverseOutput(true)
                .setSlaveSparks(List.of(new SlaveSparkMax(LEFT_LEADER_FOLLOWER_1_PORT, false))));

    var drive =
        new DriveUnidirectionalWithGyro(
            leftMaster,
            rightMaster,
            navx,
            new DriveSettingsBuilder()
                .postEncoderGearing(1 / 20.45)
                .maxSpeed(2.3)
                .leftFeedforward(new SimpleMotorFeedforward(0.24453, 5.4511, 0.7127))
                .rightFeedforward(new SimpleMotorFeedforward(0.2691, 5.3099, 0.51261))
                .build(),
            0.61755);

    var throttlePrototype =
        new ThrottlePolynomialBuilder().stick(driveJoystick).smoothingTimeSecs(0.04).scale(0.7);
    var rotThrottle =
        throttlePrototype
            .axis(0)
            .deadband(0.08)
            .inverted(false)
            .polynomial(new Polynomial(Map.of(1., 0.009, 2., 0.002), null))
            .build();
    var fwdThrottle =
        new ThrottleSum(
            new Throttle[] {
              throttlePrototype
                  .axis(3)
                  .deadband(0.05)
                  .inverted(true)
                  .polynomial(
                      new Polynomial(
                          Map.of(
                              1., 0.01, // 0.06 * x^2 + 0.01 * x^1
                              2., 0.06),
                          null))
                  .build(),
              throttlePrototype.axis(2).inverted(false).build()
            });
    var oi =
        new OIArcadeWithDPad(
            rotThrottle,
            fwdThrottle,
            0.1,
            false,
            driveJoystick,
            new Polynomial(
                Map.of(
                    0.5, 0.4,
                    0., 0.2),
                null),
            0.7,
            true);

    var defaultDriveCommand =
        new DefaultCommand(
            drive,
            new UnidirectionalNavXDefaultDrive<>(
                0,
                new Debouncer(1.5),
                0,
                1.0,
                null,
                2,
                3.0,
                false,
                0, // TODO tune pid
                0,
                0,
                new Debouncer(0.15),
                drive,
                oi,
                new RampComponent(2.0, 2.0)));

    // Elevator
    //    var elevatorPulleyMotor =
    //        new MappedSparkMax(
    //            null,
    //            null,
    //            new SmartMotorConfig()
    //                .setName("elevator")
    //                .setPort(ELEVATOR_MOTOR_PORT)
    //                .setReverseOutput(false)
    //                .setEnableBrakeMode(true)
    //                .setPdp(pdp)
    //                .setCurrentLimit(40)
    //                .setUnitPerRotation(.19244564)
    //                .setEnableVoltageComp(false)
    //                .setPostEncoderGearing(1.0 / 30)
    //                .setPerGearSettings(
    //                    List.of(
    //                        new PerGearSettingsBuilder()
    //                            .gear(Shiftable.Gear.LOW)
    //                            .maxSpeed(ELEVATOR_MAX_VELOCITY)
    //                            .build()))
    //                );
    //    // PID constants for velocity controlled elevator motor
    //    //    elevatorPulleyMotor.setPID(0.0003, 0.0000008, 0.0146);
    //    // PID constants for position controlled elevator motor
    //    //    elevatorPulleyMotor.setPID(0.2, 0.0008, 0.016);
    //    elevatorPulleyMotor.setPID(1, 0.000000, 1.000);
    //    // WE ASSUME THE ELEVATOR STARTS AT THE BOTTOM
    //    // PLEASE MAKE SURE ELEVATOR IS ACTUALLY AT THE BOTTOM

    //    var elevator =
    //        new OneMotorPulleyElevator(
    //            elevatorPulleyMotor,
    //            OneMotorPulleyElevator.ElevatorPosition.BOTTOM,
    //            new ElevatorFeedforward(0.11311, 0.15109, 3.8541, 0.30047),
    //            new TrapezoidProfile.Constraints(ELEVATOR_MAX_VELOCITY, ELEVATOR_MAX_ACCEL));
    //        var setVelocityCommand = new SetVelocity(elevator, mechanismsJoystick,
    // ELEVATOR_MAX_VELOCITY);

    var elevatorspark =
        new CANSparkMax(ELEVATOR_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);

    var updater = new Updater(List.of(pdp, oi, navx, drive));

    var defaultCommands = List.of(defaultDriveCommand);

    var buttons =
        List.of(
            // elevator move to TOP position
            //            new CommandButton(
            //                new SimpleButton(mechanismsJoystick, ELEVATOR_MOVE_TO_TOP),
            //                new MoveToPosition(OneMotorPulleyElevator.ElevatorPosition.TOP,
            // elevator),
            //                CommandButton.Action.WHEN_PRESSED),
            new CommandButton(
                new SimpleButton(mechanismsJoystick, ELEVATOR_MOVE_UP),
                new InstantCommand(() -> elevatorspark.set(0.75)),
                CommandButton.Action.WHILE_HELD),
            new CommandButton(
                new SimpleButton(mechanismsJoystick, ELEVATOR_MOVE_UP),
                new InstantCommand(() -> elevatorspark.set(0)),
                CommandButton.Action.WHEN_RELEASED),
            // elevator move to UPPER position
            //            new CommandButton(
            //                new SimpleButton(mechanismsJoystick, ELEVATOR_MOVE_TO_UPPER),
            //                new MoveToPosition(OneMotorPulleyElevator.ElevatorPosition.UPPER,
            // elevator),
            //                CommandButton.Action.WHEN_PRESSED),
            new CommandButton(
                new SimpleButton(mechanismsJoystick, ELEVATOR_MOVE_DOWN),
                new InstantCommand(() -> elevatorspark.set(-0.4)),
                CommandButton.Action.WHILE_HELD),
            new CommandButton(
                new SimpleButton(mechanismsJoystick, ELEVATOR_MOVE_DOWN),
                new InstantCommand(() -> elevatorspark.set(0)),
                CommandButton.Action.WHEN_RELEASED),
            // elevator move to LOWER position
            //            new CommandButton(
            //                new SimpleButton(mechanismsJoystick, ELEVATOR_MOVE_TO_LOWER),
            //                new MoveToPosition(OneMotorPulleyElevator.ElevatorPosition.LOWER,
            // elevator),
            //                CommandButton.Action.WHEN_PRESSED),
            // elevator move to BOTTOM position
            //            new CommandButton(
            //                new SimpleButton(mechanismsJoystick, ELEVATOR_MOVE_TO_BOTTOM),
            //                new MoveToPosition(OneMotorPulleyElevator.ElevatorPosition.BOTTOM,
            // elevator),
            //                CommandButton.Action.WHEN_PRESSED),
            // Close intake
            new CommandButton(
                new SimpleButton(mechanismsJoystick, INTAKE_CLOSE_BUTTON),
                new InstantCommand(() -> intake.set(DoubleSolenoid.Value.kForward)),
                CommandButton.Action.WHEN_PRESSED),
            // Open intake
            new CommandButton(
                new SimpleButton(mechanismsJoystick, INTAKE_OPEN_BUTTON),
                new InstantCommand(() -> intake.set(DoubleSolenoid.Value.kReverse)),
                CommandButton.Action.WHEN_PRESSED));

    var subsystems = List.<Subsystem>of(drive /*, elevator*/);

    var robotStartupCommands = List.<Command>of(new InstantCommand(drive::resetPosition));

    var autoStartupCommands =
        List.<Command>of(
            new InstantCommand(() -> elevatorspark.set(0.3))
                .andThen(new WaitCommand(3.0))
                .andThen(new InstantCommand(() -> elevatorspark.set(0))),
            new InstantCommand(() -> intake.set(DoubleSolenoid.Value.kReverse))
                .andThen(new DriveAtSpeed<>(drive, -.5, 5))
                .andThen(new WaitCommand(1.0))
                .andThen(new InstantCommand(() -> intake.set(DoubleSolenoid.Value.kForward)))
                .andThen(new WaitCommand(1.0))
                .andThen(new InstantCommand(() -> intake.set(DoubleSolenoid.Value.kReverse)))
                .andThen( // Move down a little
                    new InstantCommand(() -> elevatorspark.set(-0.3))
                        .andThen(new WaitCommand(2.35))
                        .andThen(new InstantCommand(() -> elevatorspark.set(0))))
                .andThen(new InstantCommand(() -> intake.set(DoubleSolenoid.Value.kForward)))
                .andThen(new WaitCommand(0.5))
                .andThen( // Move up a little
                    new InstantCommand(() -> elevatorspark.set(0.3))
                        .andThen(new WaitCommand(2))
                        .andThen(new InstantCommand(() -> elevatorspark.set(0))))
                .andThen(new DriveAtSpeed<>(drive, 0.2, 2))
            //                new RamseteControllerGoToPosition(
            ////                        drive,
            ////                        0.1,
            ////                        0.1,
            ////                        0.01,
            ////                        new MappedPIDController(0.001, 0, 0, "leftPIDController"),
            ////                        new MappedPIDController(0.001, 0, 0, "rightPIDController"),
            ////                        new Pose2d(new Translation2d(-3.6, 0), new Rotation2d(0)),
            ////                        List.of(),
            ////                        true)
            );
    var teleopStartupCommands = List.<Command>of();
    var testStartupCommands = List.<Command>of();
    var allCommands =
        new CommandContainer(
            defaultCommands,
            buttons,
            robotStartupCommands,
            autoStartupCommands,
            teleopStartupCommands,
            testStartupCommands);

    return new RobotMap(subsystems, pdp, updater, allCommands, joysticks, false);
  }
}
