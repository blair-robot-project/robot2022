package frc.team449;

public class Converter {
  private Converter() {
    throw new IllegalStateException("Converter is a utility class!");
  }
  /**
   * Converts a joystick input to a velocity in m/s by scaling it to the given scale.
   *
   * @param input the joystick input to be converted
   * @param scale the scale to be applied (ex: the max elevator velocity)
   * @return the input scaled by the scale
   */
  public static double joystickInputToVelocity(double input, double scale) {
    return input * scale;
  }
}