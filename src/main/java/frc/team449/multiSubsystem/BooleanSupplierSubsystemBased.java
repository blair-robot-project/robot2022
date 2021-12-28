package frc.team449.multiSubsystem;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.Objects;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/** A condition based on the condition of a conditional subsystem. */
public class BooleanSupplierSubsystemBased extends BooleanSupplierBooleanSupplierBased {
  /**
   * Default constructor.
   *
   * @param subsystem The conditional subsystem on whose condition the condition depends.
   * @param useCached If true, use {@link SubsystemConditional#isConditionTrueCached()}; otherwise,
   *     use {@link SubsystemConditional#isConditionTrue()}
   */
  @JsonCreator
  public BooleanSupplierSubsystemBased(
      @NotNull @JsonProperty(required = true) final SubsystemConditional subsystem,
      final @Nullable Boolean useCached) { // TODO useCached probably shouldn't be nullable
    super(
        Objects.requireNonNullElse(useCached, true)
            ? subsystem::isConditionTrueCached
            : subsystem::isConditionTrue);
  }
}
