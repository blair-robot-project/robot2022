package frc.team449.multiSubsystem;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.other.Util;
import io.github.oblarg.oblog.Loggable;
import java.util.Map;
import org.jetbrains.annotations.NotNull;

/** A simple intake subsystem that relies on a single motor to rotate some part of it. */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class IntakeSimple extends SubsystemBase
    implements SubsystemIntake, MotorController, Loggable {

  /** The motor this subsystem controls. */
  @NotNull private final MotorController motor;

  /**
   * The velocities for the motor to go at for each of the modes, on [-1, 1]. Can be null to
   * indicate that this intake doesn't have/use that mode.
   */
  @NotNull private final Map<IntakeMode, Double> velocities;

  /** The current mode. */
  @NotNull private SubsystemIntake.IntakeMode mode;

  /**
   * Default constructor
   *
   * @param motor The motor this subsystem controls.
   * @param velocities The velocity for the motor to go at for each {@link IntakeMode}, on the
   *     interval [-1, 1]. Modes can be missing to indicate that this intake doesn't have/use them.
   */
  @JsonCreator
  public IntakeSimple(
      @NotNull @JsonProperty(required = true) final MotorController motor,
      @NotNull @JsonProperty(required = true) final Map<IntakeMode, Double> velocities) {

    this.motor = motor;
    this.velocities = velocities;

    this.mode = IntakeMode.OFF;

    if (velocities.containsKey(IntakeMode.OFF))
      System.err.println(
          Util.getLogPrefix(this)
              + "Warning: velocity for mode "
              + IntakeMode.OFF
              + " will be ignored.");

    if (velocities.isEmpty()) {
      System.err.println(
          Util.getLogPrefix(this) + "Warning: no defined velocities; motor will never spin.");
    }
  }

  /** @return the current mode of the intake. */
  @NotNull
  @Override
  public SubsystemIntake.IntakeMode getMode() {
    return this.mode;
  }

  /** @param mode The mode to switch the intake to. */
  @Override
  public void setMode(@NotNull final SubsystemIntake.IntakeMode mode) {
    //  This guard means intake instances should not share motors.
    if (this.getMode() == mode) return;

    if (mode == IntakeMode.OFF) {
      this.mode = IntakeMode.OFF;
      motor.set(0);
      motor.disable();
    } else if (this.velocities.containsKey(mode)) {
      this.mode = mode;
      this.motor.set(this.velocities.get(mode));
    } else {
      DriverStation.reportError("Mode not defined for instance: " + mode, false);
    }
  }

  /**
   * Set output to a given input.
   *
   * @param input The input to give to the motor.
   */
  @Override
  public void set(final double input) {
    this.motor.set(input);
  }

  @Override
  public double get() {
    return this.motor.get();
  }

  @Override
  public void setInverted(boolean isInverted) {
    this.motor.setInverted(isInverted);
  }

  @Override
  public boolean getInverted() {
    return this.motor.getInverted();
  }

  /** Disable the motor. */
  @Override
  public void disable() {
    this.motor.disable();
  }

  @Override
  public void stopMotor() {
    this.motor.stopMotor();
  }
}
