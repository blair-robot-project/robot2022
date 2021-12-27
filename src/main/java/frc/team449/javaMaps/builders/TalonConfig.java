package frc.team449.javaMaps.builders;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import frc.team449.jacksonWrappers.SlaveTalon;
import frc.team449.jacksonWrappers.SlaveVictor;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Motor controller configuration, along with Talon-specific stuff */
public class TalonConfig extends MotorConfig<TalonConfig> {

  private final Map<ControlFrame, Integer> controlFrameRatesMillis = new HashMap<>();
  private final Map<StatusFrameEnhanced, Integer> statusFrameRatesMillis = new HashMap<>();
  private int voltageCompSamples = 32;
  private @Nullable FeedbackDevice feedbackDevice;
  private boolean reverseSensor = false;
  private final List<SlaveTalon> slaveTalons = new ArrayList<>();
  private final List<SlaveVictor> slaveVictors = new ArrayList<>();

  @NotNull
  public Map<ControlFrame, Integer> getControlFrameRatesMillis() {
    return new HashMap<>(this.controlFrameRatesMillis);
  }

  public TalonConfig addControlFrameRateMillis(@NotNull ControlFrame controlFrame, int updateRate) {
    this.controlFrameRatesMillis.put(controlFrame, updateRate);
    return this;
  }

  @NotNull
  public Map<StatusFrameEnhanced, Integer> getStatusFrameRatesMillis() {
    return new HashMap<>(this.statusFrameRatesMillis);
  }

  public TalonConfig addStatusFrameRateMillis(
      @NotNull StatusFrameEnhanced statusFrame, int updateRate) {
    this.statusFrameRatesMillis.put(statusFrame, updateRate);
    return this;
  }

  public int getVoltageCompSamples() {
    return this.voltageCompSamples;
  }

  public TalonConfig setVoltageCompSamples(int voltageCompSamples) {
    this.voltageCompSamples = voltageCompSamples;
    return this;
  }

  @Nullable
  public FeedbackDevice getFeedbackDevice() {
    return feedbackDevice;
  }

  public TalonConfig setFeedbackDevice(@NotNull FeedbackDevice feedbackDevice) {
    this.feedbackDevice = feedbackDevice;
    return this;
  }

  public boolean getReverseSensor() {
    return reverseSensor;
  }

  public TalonConfig setReverseSensor(boolean reverseSensor) {
    this.reverseSensor = reverseSensor;
    return this;
  }

  @NotNull
  public List<SlaveTalon> getSlaveTalons() {
    return new ArrayList<>(slaveTalons);
  }

  public TalonConfig addSlaveTalon(@NotNull SlaveTalon slaveTalon) {
    this.slaveTalons.add(slaveTalon);
    return this;
  }

  @NotNull
  public List<SlaveVictor> getSlaveVictors() {
    return new ArrayList<>(slaveVictors);
  }

  public TalonConfig addSlaveVictor(@NotNull SlaveVictor slaveVictor) {
    this.slaveVictors.add(slaveVictor);
    return this;
  }

  public TalonConfig copy() {
    var copy =
        new TalonConfig()
            .setFeedbackDevice(this.getFeedbackDevice())
            .setReverseSensor(this.getReverseSensor())
            .setVoltageCompSamples(this.getVoltageCompSamples());
    this.copyTo(copy);

    copy.controlFrameRatesMillis.putAll(this.controlFrameRatesMillis);
    copy.statusFrameRatesMillis.putAll(this.statusFrameRatesMillis);
    copy.slaveTalons.addAll(this.getSlaveTalons());
    copy.slaveVictors.addAll(this.getSlaveVictors());

    return copy;
  }
}
