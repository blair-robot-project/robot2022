package frc.team449.wrappers.simulated;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.team449.wrappers.AHRS;
import org.jetbrains.annotations.NotNull;

/** A simulated AHRS. See https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/ */
public class AHRSSim extends AHRS {
  /** The default name used for the simulated NavX (probably the one you want */
  public static final String DEFAULT_NAVX_NAME = "navX-Sensor[0]";

  private final SimBoolean isConnected;
  private final SimDouble yaw;
  private final SimDouble pitch;
  private final SimDouble roll;
  private final SimDouble compassHeading;
  private final SimDouble fusedHeading;
  private final SimDouble linearWorldAccelX;
  private final SimDouble linearWorldAccelY;
  private final SimDouble linearWorldAccelZ;

  /**
   * Default constructor.
   *
   * @param port The port the NavX is plugged into. It seems like only kMXP (the port on the RIO)
   *     works.
   * @param invertYaw Whether or not to invert the yaw axis. Defaults to true.
   * @param devName The name of the simulated device. If you don't know what this should be, try
   *     {@link AHRSSim#AHRSSim(SerialPort.Port, boolean)}, where it defaults to {@link
   *     AHRSSim#DEFAULT_NAVX_NAME}
   */
  public AHRSSim(SerialPort.Port port, boolean invertYaw, @NotNull String devName) {
    super(port, invertYaw);
    int devHandle = SimDeviceDataJNI.getSimDeviceHandle(devName);
    this.isConnected = new SimBoolean(SimDeviceDataJNI.getSimValueHandle(devHandle, "Connected"));
    this.yaw = new SimDouble(SimDeviceDataJNI.getSimValueHandle(devHandle, "Yaw"));
    this.pitch = new SimDouble(SimDeviceDataJNI.getSimValueHandle(devHandle, "Pitch"));
    this.roll = new SimDouble(SimDeviceDataJNI.getSimValueHandle(devHandle, "Roll"));
    this.compassHeading =
        new SimDouble(SimDeviceDataJNI.getSimValueHandle(devHandle, "CompassHeading"));
    this.fusedHeading =
        new SimDouble(SimDeviceDataJNI.getSimValueHandle(devHandle, "FusedHeading"));
    this.linearWorldAccelX =
        new SimDouble(SimDeviceDataJNI.getSimValueHandle(devHandle, "LinearWorldAccelX"));
    this.linearWorldAccelY =
        new SimDouble(SimDeviceDataJNI.getSimValueHandle(devHandle, "LinearWorldAccelY"));
    this.linearWorldAccelZ =
        new SimDouble(SimDeviceDataJNI.getSimValueHandle(devHandle, "LinearWorldAccelZ"));
    setHeading(0);
  }

  /**
   * Alternative constructor where the device name is taken to be {@link AHRSSim#DEFAULT_NAVX_NAME},
   * which is probably what you want
   *
   * @param port The port the NavX is plugged into. It seems like only kMXP (the port on the RIO) *
   *     works.
   * @param invertYaw Whether or not to invert the yaw axis. Defaults to true.
   */
  public AHRSSim(SerialPort.Port port, boolean invertYaw) {
    this(port, invertYaw, DEFAULT_NAVX_NAME);
  }

  @Override
  public double getHeading() {
    return invertYaw * yaw.get();
  }

  @Override
  public void setHeading(double headingDegrees) {
    // todo this is kind of a hack to get around yaw being unitialized
    //   when super() calls setHeading(0)
    if (yaw != null) {
      yaw.set(headingDegrees * invertYaw);
    }
  }

  @Override
  public double getXAccel() {
    return gsToMetersPerSecondSquared(linearWorldAccelX.get());
  }

  @Override
  public double getYAccel() {
    return gsToMetersPerSecondSquared(linearWorldAccelY.get());
  }

  @Override
  public double getPitch() {
    return pitch.get();
  }
}
