package frc.team449._2020.feeder;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import frc.team449._2020.multiSubsystem.IntakeSimple;
import frc.team449.generalInterfaces.simpleMotor.SimpleMotor;
import org.jetbrains.annotations.NotNull;

import java.util.Map;

@JsonTypeInfo(
    use = JsonTypeInfo.Id.CLASS,
    include = JsonTypeInfo.As.WRAPPER_OBJECT,
    property = "@class")
public class FeederSystem extends IntakeSimple {

  /** Whether the default indexing system is being overridden by having moved backwards */
  private boolean overriding = false;

  /**
   * Default constructor
   *
   * @param motor The motor this subsystem controls.
   * @param velocities The velocity for the motor to go at for each {@link IntakeMode}, on the
   */
  @JsonCreator
  public FeederSystem(
      @NotNull final SimpleMotor motor, @NotNull final Map<IntakeMode, Double> velocities) {
    super(motor, velocities);
  }

  @Override
  public void setMode(@NotNull final IntakeMode mode) {
    if (mode.equals(IntakeMode.OUT_SLOW)) {
      overriding = true;
    }
    super.setMode(mode);
  }

  public boolean isOverriding() {
    return overriding;
  }

  public void setOverriding(final boolean overriding) {
    this.overriding = overriding;
  }
}
