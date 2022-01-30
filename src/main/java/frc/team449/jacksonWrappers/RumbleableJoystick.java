package frc.team449.jacksonWrappers;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import frc.team449.generalInterfaces.rumbleable.Rumbleable;
import frc.team449.jacksonWrappers.simulated.JoystickSimulated;

import static frc.team449.other.Util.getLogPrefix;

/** A Jackson-compatible wrapper on a {@link Joystick}. */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class RumbleableJoystick extends Joystick implements Rumbleable {
  /**
   * Whether to construct instances of {@link JoystickSimulated} instead of {@link
   * RumbleableJoystick} when the robot is running in a simulation.
   */
  private static final boolean SIMULATE = true;

  /**
   * Default constructor
   *
   * @param port The USB port of this joystick, on [0, 5].
   */
  public RumbleableJoystick(@JsonProperty(required = true) final int port) {
    super(port);
  }

  /**
   * Factory method to enable faking in simulation.
   *
   * @param port The USB port of this joystick, on [0, 5].
   */
  @JsonCreator
  public static RumbleableJoystick create(@JsonProperty(required = true) final int port) {
    if (!SIMULATE || RobotBase.isReal()) {
      return new RumbleableJoystick(port);
    }

    System.out.println(
        getLogPrefix(RumbleableJoystick.class) + "Creating simulated joystick on port " + port);
    return new JoystickSimulated(port);
  }

  /**
   * Rumble at a given strength on each side of the device.
   *
   * @param left The strength to rumble the left side, on [-1, 1]
   * @param right The strength to rumble the right side, on [-1, 1]
   */
  @Override
  public void rumble(final double left, final double right) {
    this.setRumble(RumbleType.kLeftRumble, left);
    this.setRumble(RumbleType.kRightRumble, right);
  }
}
