package frc.team449.components;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Color;
import org.jetbrains.annotations.Nullable;

@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class LEDComponent {

  final AddressableLED LEDStrip;

  final AddressableLEDBuffer buffer;

  // todo array to hold HSV values of each LED

  @JsonCreator
  public LEDComponent(@JsonProperty(required = true) int port, @Nullable Integer LEDCount) {
    LEDStrip = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(LEDCount != null ? LEDCount : 10);

    LEDStrip.setLength(buffer.getLength());
  }

  public void turnStripOn() {
    LEDStrip.start();
    Shuffleboard.addEventMarker("LED Controller", "LED's on!", EventImportance.kNormal);
  }

  public void turnStripOff() {
    LEDStrip.stop();
    Shuffleboard.addEventMarker("LED Controller", "LED's off!", EventImportance.kNormal);
  }

  /**
   * Set a specific part of the LED to the given RGB/HSV values
   *
   * @param lowerBound The start of the range (0 or greater)
   * @param upperBound The end of the range (can be at most {@code buffer}'s length)
   * @param v1 r if RGB, h if HSV
   * @param v2 g if RGB, s if HSV
   * @param v3 b if RGB, v if HSV
   * @param colorModel RGB or HSV
   */
  private void setSpecificRange(
      int lowerBound, int upperBound, int v1, int v2, int v3, ColorModel colorModel) {
    int checkedLowerBound = Math.max(lowerBound, 0);
    int checkedUpperBound = Math.min(upperBound, buffer.getLength());

    if (upperBound > buffer.getLength()) {
      Shuffleboard.addEventMarker(
          "LED Controller",
          "Set range larger than set strip length! defaulting to set strip length",
          EventImportance.kTrivial);
    }

    if (colorModel == ColorModel.RGB) {
      for (int i = checkedLowerBound; i < checkedUpperBound; i++) {
        buffer.setRGB(i, v1, v2, v3);
      }
    } else if (colorModel == ColorModel.HSV) {
      for (int i = checkedLowerBound; i < checkedUpperBound; i++) {
        buffer.setHSV(i, v1, v2, v3);
      }
    }
  }

  public void setStripColor(Color color) {
    setStrip(
        (int) (color.red * 255),
        (int) (color.green * 255),
        (int) (color.blue * 255),
        ColorModel.RGB);
  }

  private void setStrip(int v1, int v2, int v3, ColorModel colorModel) {
    setSpecificRange(0, buffer.getLength(), v1, v2, v3, colorModel);
  }

  public int[] RGBtoHSV(int r, int g, int b) {
    int[] storage = new int[3];
    // todo this
    storage[0] = r;
    storage[1] = g;
    storage[2] = b;

    return storage;
  }

  private enum ColorModel {
    RGB,
    HSV
  }
}
