package frc.team449.javaMaps.builders;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.cscore.UsbCamera;
import org.jetbrains.annotations.NotNull;

/** A utility class to make {@link UsbCamera}'s. */
public final class UsbCameraCreator {

  private UsbCameraCreator() {}

  /**
   * Default constructor
   *
   * @param name The human-friendly name for this camera.
   * @param devAddress The address of this device in /dev/, e.g. this would be 0 for /dev/video0, 1
   *     for /dev/video1.
   * @param width The width of this camera's output, in pixels. There's a minimum value for this
   *     that WPILib won't let us go below, but I don't know what it is.
   * @param height The height of this camera's output, in pixels. There's a minimum value for this
   *     that WPILib won't let us go below, but I don't know what it is.
   * @param fps The frames per second this camera tries to transmit. There's a minimum value for
   *     this that WPILib won't let us go below, but I don't know what it is.
   */
  @JsonCreator
  public static UsbCamera createUsbCamera(
      @NotNull String name, int devAddress, int width, int height, int fps) {
    var camera = new UsbCamera(name, devAddress);
    camera.setResolution(width, height);
    camera.setFPS(fps);

    // If we don't have the exposure be automatic, the camera will be super laggy. No idea why.
    camera.setExposureAuto();

    return camera;
  }
}
