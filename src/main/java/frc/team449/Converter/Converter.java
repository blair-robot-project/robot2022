package frc.team449.Converter;

public class Converter {
    public static double joystickInputToVelocity(double input, double scale) {
        double output = input * scale;
        return output;
    }
}
