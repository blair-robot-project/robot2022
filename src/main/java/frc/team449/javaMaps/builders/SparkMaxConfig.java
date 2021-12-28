package frc.team449.javaMaps.builders;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import java.util.HashMap;
import java.util.Map;
import org.jetbrains.annotations.Nullable;

/** Motor controller configuration, along with some Spark-specific stuff */
public final class SparkMaxConfig extends MotorConfig<SparkMaxConfig> {
  private final Map<CANSparkMax.PeriodicFrame, Integer> statusFrameRatesMillis = new HashMap<>();
  private @Nullable Integer controlFrameRateMillis;

  @Nullable
  public Integer getControlFrameRateMillis() {
    return this.controlFrameRateMillis;
  }

  public SparkMaxConfig setControlFrameRateMillis(int controlFrameRateMillis) {
    this.controlFrameRateMillis = controlFrameRateMillis;
    return this;
  }

  public Map<CANSparkMax.PeriodicFrame, Integer> getStatusFrameRatesMillis() {
    return new HashMap<>(this.statusFrameRatesMillis);
  }

  public SparkMaxConfig addStatusFrameRateMillis(
      CANSparkMaxLowLevel.PeriodicFrame frame, int rate) {
    this.statusFrameRatesMillis.put(frame, rate);
    return this;
  }

  public SparkMaxConfig copy() {
    var copy = new SparkMaxConfig();
    this.copyTo(copy);

    if (this.controlFrameRateMillis != null) {
      copy.setControlFrameRateMillis(this.controlFrameRateMillis);
    }

    copy.statusFrameRatesMillis.putAll(this.getStatusFrameRatesMillis());

    return copy;
  }
}
