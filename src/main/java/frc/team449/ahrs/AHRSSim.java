package frc.team449.ahrs;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import org.jetbrains.annotations.NotNull;

/** A simulated AHRS. See https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/ */
public class AHRSSim extends AHRS {
  /** The default name used for the simulated NavX (probably the one you want */
  public static final String DEFAULT_NAVX_NAME = "navX-Sensor";

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
   * @param index The NavX index. If you don't know what this should be, try {@link
   *     AHRSSim#AHRSSim(SerialPort.Port, boolean)}, where it defaults to 0
   */
  public AHRSSim(SerialPort.Port port, boolean invertYaw, @NotNull String devName, int index) {
    super(port, invertYaw);

    var deviceSim = new SimDeviceSim(devName, index);
    this.isConnected = deviceSim.getBoolean("Connected");
    this.yaw = deviceSim.getDouble("Yaw");
    this.pitch = deviceSim.getDouble("Pitch");
    this.roll = deviceSim.getDouble("Roll");
    this.compassHeading = deviceSim.getDouble("CompassHeading");
    this.fusedHeading = deviceSim.getDouble("FusedHeading");
    this.linearWorldAccelX = deviceSim.getDouble("LinearWorldAccelX");
    this.linearWorldAccelY = deviceSim.getDouble("LinearWorldAccelY");
    this.linearWorldAccelZ = deviceSim.getDouble("LinearWorldAccelZ");
    setHeading(new Rotation2d());
  }

  /**
   * Alternative constructor where the device name is taken to be {@link AHRSSim#DEFAULT_NAVX_NAME},
   * and the index is taken to be 0, which is probably what we want
   *
   * @param port The port the NavX is plugged into. It seems like only kMXP (the port on the RIO) *
   *     works.
   * @param invertYaw Whether or not to invert the yaw axis. Defaults to true.
   */
  public AHRSSim(SerialPort.Port port, boolean invertYaw) {
    this(port, invertYaw, DEFAULT_NAVX_NAME, 0);
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
