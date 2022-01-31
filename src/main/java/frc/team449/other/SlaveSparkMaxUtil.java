package frc.team449.other;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxLimitSwitch;
import org.jetbrains.annotations.NotNull;

/** Helps create {@link com.revrobotics.CANSparkMax}'s that'll be slaved to other sparks */
public final class SlaveSparkMaxUtil {

  private SlaveSparkMaxUtil() {}

  @JsonCreator
  public static CANSparkMax createSlaveSpark(int port) {
    var slaveSpark = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushless);

    slaveSpark
        .getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)
        .enableLimitSwitch(false);
    slaveSpark
        .getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)
        .enableLimitSwitch(false);

    slaveSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    slaveSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);
    slaveSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100);

    return slaveSpark;
  }

  public static void setMasterSpark(
      @NotNull CANSparkMax slaveSpark,
      @NotNull CANSparkMax masterController,
      boolean brakeMode,
      boolean inverted) {
    slaveSpark.follow(masterController, inverted);
    slaveSpark.setIdleMode(brakeMode ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }
}
