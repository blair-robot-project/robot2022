package frc.team449;

public class Converter {
    private Converter() {
        throw new IllegalStateException("Converter is a utility class!");
    }
    public static double joystickInputToVelocity(double input, double scale) {
        return input * scale;
    }
}
