package frc.team449.auto.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.components.TrajectoryGenerationComponent;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import org.jetbrains.annotations.NotNull;

import java.util.Objects;

public class RamseteControllerUnidirectionalDrive extends CommandBase {

    DriveUnidirectionalWithGyro drivetrain;
    RamseteController ramseteFeedback;
    PIDController leftController, rightController;
    DifferentialDriveWheelSpeeds previousSpeeds;
    Trajectory trajectory;
    SimpleMotorFeedforward rightFF;
    SimpleMotorFeedforward leftFF;
    Field2d field;
    double absoluteTime;
    double relativeTime;

    public RamseteControllerUnidirectionalDrive(@NotNull DriveUnidirectionalWithGyro drivetrain,
                                                @NotNull double P,
                                                @NotNull double D,
                                                @NotNull TrajectoryGenerationComponent trajectoryGenerator,
                                                @NotNull SimpleMotorFeedforward rightFF,
                                                @NotNull SimpleMotorFeedforward leftFF,
                                                Field2d field){
        this.drivetrain = drivetrain;
        ramseteFeedback = new RamseteController();
        leftController = new PIDController(P, 0, D);
        rightController = new PIDController(P, 0, D);
        this.rightFF = rightFF;
        this.leftFF = leftFF;
        trajectory = trajectoryGenerator.getTrajectory();
        this.field = field;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
//        if (field != null)
//            field.getObject("traj").setTrajectory(trajectory);
        absoluteTime = Timer.getFPGATimestamp();
        Trajectory.State initialState = trajectory.sample(Timer.getFPGATimestamp() - absoluteTime);
        previousSpeeds = drivetrain.getDriveKinematics().toWheelSpeeds(new ChassisSpeeds(initialState.velocityMetersPerSecond,
                0,
                (initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond)));
        leftController.reset();
        rightController.reset();
    }

    @Override
    public void execute(){
        relativeTime = Timer.getFPGATimestamp();
        DifferentialDriveWheelSpeeds targetWheelSpeeds = drivetrain.getDriveKinematics().toWheelSpeeds(
                ramseteFeedback.calculate(drivetrain.getCurrentPose(), trajectory.sample(Timer.getFPGATimestamp() - absoluteTime)));
        DifferentialDriveWheelSpeeds currentWheelSpeeds = drivetrain.getWheelSpeeds();

        double leftTarget = targetWheelSpeeds.leftMetersPerSecond;
        double rightTarget = targetWheelSpeeds.rightMetersPerSecond;
        double leftCurrent = currentWheelSpeeds.leftMetersPerSecond;
        double rightCurrent = currentWheelSpeeds.leftMetersPerSecond;

        double leftFeedforward = leftFF.calculate(leftTarget,
                (leftTarget - leftCurrent) / (Timer.getFPGATimestamp() - relativeTime));
        double rightFeedforward = rightFF.calculate(rightTarget,
                (rightTarget - rightCurrent) / (Timer.getFPGATimestamp() - relativeTime));

        double leftOutput = leftFeedforward + leftController.calculate(leftCurrent, leftTarget);
        double rightOutput = rightFeedforward + rightController.calculate(rightCurrent, rightTarget);

        drivetrain.setVoltage(leftOutput, rightOutput);

    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - absoluteTime) >= trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted){
            Shuffleboard.addEventMarker("Ramsete controller interrupted! Stopping the robot.", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        drivetrain.fullStop();
        Shuffleboard.addEventMarker("Ramsete controller end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
}