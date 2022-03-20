package frc.team449.wrappers;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Wrapper for an {@link AnalogInput} pressure sensor that returns a voltage linearly proportional
 * to pressure.
 */
public class PressureSensor {

  /** The AnalogInput this is a wrapper on. */
  private final AnalogInput sensor;

  /**
   * Default constructor.
   *
   * @param port The port of the sensor.
   * @param oversampleBits The number of oversample bits.
   * @param averageBits The number of averaging bits.
   */
  public PressureSensor(int port, int oversampleBits, int averageBits) {
    sensor = new AnalogInput(port);
    sensor.setOversampleBits(oversampleBits);
    sensor.setAverageBits(averageBits);
  }

  /**
   * Returns the pressure measured by the sensor.
   *
   * @return pressure in PSI
   */
  public double getPressure() {
    // these are constants given by REV, assuming 5.0V in
    return 50.0 * sensor.getAverageVoltage() - 25.0;
  }
}
