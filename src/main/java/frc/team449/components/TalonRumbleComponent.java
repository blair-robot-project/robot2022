package frc.team449.components;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import frc.team449.generalInterfaces.rumbleable.Rumbleable;
import frc.team449.jacksonWrappers.MappedTalon;

import javax.annotation.Nullable;

@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class TalonRumbleComponent implements Runnable {

  final MappedTalon talon;
  final Rumbleable joystick;
  final boolean inverted;
  final Double rumbleAmount;

  @JsonCreator
  public TalonRumbleComponent(
      @JsonProperty(required = true) MappedTalon talon,
      @JsonProperty(required = true) Rumbleable joystick,
      @Nullable Double rumbleAmount,
      boolean inverted) {
    this.talon = talon;
    this.joystick = joystick;
    this.inverted = inverted;
    this.rumbleAmount = rumbleAmount != null ? rumbleAmount : 1;
  }

  @Override
  public void run() {
    if (this.talon.isInhibitedForward() ^ this.inverted) {
      this.joystick.rumble(0, this.rumbleAmount);
    } else if (this.talon.isInhibitedReverse() ^ this.inverted) {
      this.joystick.rumble(this.rumbleAmount, 0);
    } else {
      this.joystick.rumble(0, 0);
    }
  }
}
