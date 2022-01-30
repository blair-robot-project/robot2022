package frc.team449.other;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.team449.other.SlaveVictorUtils;
import org.jetbrains.annotations.Nullable;

import java.util.List;

/** A helper static class to create {@link VictorSP}'s and {@link WPI_VictorSPX}'s. */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public final class VictorCreator {
  private VictorCreator() {}

  public static MotorController createVictorSP(int port, boolean inverted) {
    var victorSP = new VictorSP(port);
    victorSP.setInverted(inverted);
    return victorSP;
  }

  public static MotorController createVictorSPX(
      int port,
      boolean brakeMode,
      boolean inverted,
      boolean enableVoltageComp,
      @Nullable Double peakVoltageForward,
      @Nullable Double peakVoltageRev,
      @Nullable Integer voltageCompSamples,
      @Nullable List<VictorSPX> slaveVictors) {
    var victorSPX = new WPI_VictorSPX(port);
    int voltComp = voltageCompSamples != null ? voltageCompSamples : 32;
    victorSPX.setInverted(inverted);
    victorSPX.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
    victorSPX.enableVoltageCompensation(enableVoltageComp);
    victorSPX.configVoltageCompSaturation(12, 0);
    victorSPX.configVoltageMeasurementFilter(voltComp, 0);
    victorSPX.configPeakOutputForward(peakVoltageForward != null ? peakVoltageForward / 12. : 1, 0);
    victorSPX.configPeakOutputReverse(peakVoltageRev != null ? peakVoltageRev / 12. : -1, 0);

    if (slaveVictors != null) {
      for (var slave : slaveVictors) {
        SlaveVictorUtils.setMaster(
            slave, victorSPX, brakeMode, enableVoltageComp ? voltComp : null);
      }
    }
    return victorSPX;
  }
}
