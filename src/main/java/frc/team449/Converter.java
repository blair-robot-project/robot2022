package frc.team449;

public class Converter {
  private Converter() {
    throw new IllegalStateException("Converter is a utility class!");
  }
  /**
   * Converts a joystick input to a velocity in m/s by multiplying it by it's absolute value (to
   * preserve negativity) and then scaling it to the given scale.
   *
   * @param input the joystick input to be converted
   * @param scale the scale to be applied (ex: the max elevator velocity)
   * @return the input scaled by the scale
   */
  public static double joystickInputToVelocity(double input, double scale) {
    return (input * Math.abs(input)) * scale;
  }
}
