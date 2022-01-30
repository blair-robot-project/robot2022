package frc.team449.jacksonWrappers;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxLimitSwitch;
import frc.team449.generalInterfaces.SlaveMotor;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.Nullable;

public class SlaveSparkMax implements SlaveMotor, Loggable {
  /** The SparkMAX this object wraps. */
  private final CANSparkMax slaveSpark;

  private final boolean inverted;

  @JsonCreator
  public SlaveSparkMax(
      @JsonProperty(required = true) final int port, boolean inverted) {
    this.slaveSpark = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushless);

    this.inverted = inverted;

    this.slaveSpark
        .getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)
        .enableLimitSwitch(false);
    this.slaveSpark
        .getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)
        .enableLimitSwitch(false);

    this.slaveSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    this.slaveSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);
    this.slaveSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100);
  }

  public void setMasterSpark(final CANSparkMax masterController, final boolean brakeMode) {
    this.slaveSpark.follow(masterController, this.inverted);
    this.slaveSpark.setIdleMode(
        brakeMode ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }

  @Log
  public double getOutputCurrent() {
    return this.slaveSpark.getOutputCurrent();
  }

  @Log
  public double getMotorOutputVoltage() {
    return this.slaveSpark.getAppliedOutput();
  }
}
