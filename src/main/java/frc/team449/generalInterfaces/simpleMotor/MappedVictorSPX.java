package frc.team449.generalInterfaces.simpleMotor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import frc.team449.jacksonWrappers.SlaveVictor;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/** A simple wrapper on the {@link VictorSPX}. */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class MappedVictorSPX implements SimpleMotor, Loggable {

  /** The Victor SPX this object is a wrapper on. */
  @NotNull private final WPI_VictorSPX victorSPX;

  /**
   * Default constructor.
   *
   * @param port The CAN ID of this Victor SPX.
   * @param brakeMode Whether to have the Victor brake or coast when no voltage is applied.
   * @param inverted Whether or not to invert this Victor. Defaults to false.
   * @param enableVoltageComp Whether or not to enable voltage compensation. Defaults to false.
   * @param voltageCompSamples The number of 1-millisecond samples to use for voltage compensation.
   *     Defaults to 32.
   * @param slaveVictors Any other Victor SPXs slaved to this one.
   */
  @JsonCreator
  public MappedVictorSPX(
      @JsonProperty(required = true) final int port,
      @JsonProperty(required = true) final boolean brakeMode,
      final boolean inverted,
      final boolean enableVoltageComp,
      final Double peakVoltageForward,
      final Double peakVoltageRev,
      @Nullable final Integer voltageCompSamples,
      @Nullable final List<SlaveVictor> slaveVictors) {
    victorSPX = new WPI_VictorSPX(port);
    victorSPX.setInverted(inverted);
    victorSPX.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
    victorSPX.enableVoltageCompensation(enableVoltageComp);
    victorSPX.configVoltageCompSaturation(12, 0);
    victorSPX.configVoltageMeasurementFilter(
        voltageCompSamples != null ? voltageCompSamples : 32, 0);
    victorSPX.configPeakOutputForward(peakVoltageForward != null ? peakVoltageForward / 12. : 1, 0);
    victorSPX.configPeakOutputReverse(peakVoltageRev != null ? peakVoltageRev / 12. : -1, 0);

    if (slaveVictors != null) {
      // Set up slaves.
      for (final SlaveVictor slave : slaveVictors) {
        int i = voltageCompSamples != null ? voltageCompSamples : 32;
        slave.setMaster(victorSPX, brakeMode, enableVoltageComp ? i : null);
      }
    }
  }

  /**
   * Set the velocity for the motor to go at.
   *
   * @param velocity the desired velocity, on [-1, 1].
   */
  @Override
  public void setVelocity(final double velocity) {
    victorSPX.set(velocity);
  }

  @Override
  public double get() {
    return victorSPX.get();
  }

  @Override
  public void setInverted(boolean isInverted) {
    victorSPX.setInverted(isInverted);
  }

  @Override
  public boolean getInverted() {
    return victorSPX.getInverted();
  }

  /** Disables the motor, if applicable. */
  @Override
  public void disable() {
    victorSPX.set(ControlMode.Disabled, 0);
  }

  @Override
  public void stopMotor() {
    this.set(0);
  }

  @Log
  public double getBusVoltage() {
    return victorSPX.getBusVoltage();
  }

  @Log
  public double getMotorOutputVoltage() {
    return victorSPX.getMotorOutputVoltage();
  }
}
