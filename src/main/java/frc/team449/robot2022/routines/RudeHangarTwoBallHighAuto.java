package frc.team449.robot2022.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team449.ahrs.PIDAngleController;
import frc.team449.drive.unidirectional.DriveUnidirectionalWithGyro;
import frc.team449.robot2022.cargo.Cargo2022;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.function.Supplier;

public class RudeHangarTwoBallHighAuto {


  public static Command createCommand(
      @NotNull DriveUnidirectionalWithGyro drive,
      @NotNull Cargo2022 cargo,
      @NotNull Supplier<TrajectoryConfig> trajConfig,
      Field2d field) {
    return HangarTwoBallHigh.createCommand(drive, cargo, trajConfig, field);
  }
}
