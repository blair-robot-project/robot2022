package frc.team449.generalInterfaces.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class Limelight extends SubsystemBase implements Loggable {

  /** whether the limelight can see a valid target */
  private final NetworkTableEntry validTargetTable;
  /** x offset. in degrees, of the target from the crosshair */
  private final NetworkTableEntry xTable;
  /** y offset. in degrees, of the target from the crosshair */
  private final NetworkTableEntry yTable;
  /** area of the target, as a percent of the camera screen */
  private final NetworkTableEntry areaTable;
  /** rotation (-90 to 0, in degrees) of the target (as the limelight sees it) */
  private final NetworkTableEntry skewTable;
  /** the pipeline's latency contribution, in ms */
  private final NetworkTableEntry latencyTable;
  /** sidelength of the shortest side of the vision target box, in pixels */
  private final NetworkTableEntry shortTable;
  /** sidelength of the longest side of the vision target box, in pixels */
  private final NetworkTableEntry longTable;
  /** width of target box, in pixels */
  private final NetworkTableEntry widthTable;
  /** height of target box, in pixels */
  private final NetworkTableEntry heightTable;
  /** pipeline index of the limelight */
  private final NetworkTableEntry pipeTable;
  /** camtran, for getting 3D pos */
  private final NetworkTableEntry camtran;
  /** Average HSV color under the crosshair region as a NumberArray */
  private final NetworkTableEntry cTable;
  /**
   * entry to change led mode:
   *
   * <ul>
   *   <li>0 - use the LED mode in the current pipeline
   *   <li>1 - force off
   *   <li>2 - force blink
   *   <li>3 - force on
   * </ul>
   */
  private final NetworkTableEntry ledModeSet;
  /** entry to change camera mode: 0 for vision processor, 1 for driver camera */
  private final NetworkTableEntry camModeSet;
  /** entry to change pipeline (1 to 9) */
  private final NetworkTableEntry pipelineSet;
  /**
   * entry to change streaming mode
   *
   * <ul>
   *   <li>0 Standard - Side-by-side streams if a webcam is attached to Limelight
   *   <li>1 PiP Main - The secondary camera stream is placed in the lower-right corner of the
   *       primary camera stream
   *   <li>2 PiP Secondary - The primary camera stream is placed in the lower-right corner of the
   *       secondary camera stream
   * </ul>
   */
  private final NetworkTableEntry streamSet;
  /** Entry to allow taking snapshots: 0 - stop taking snapshots, 1 - take 2 snapshots per sec */
  private final NetworkTableEntry snapshotSet;

  /** pipeline for driver camera */
  private final int driverPipeline;

  // Cached values for the most recent state of the limelight while it was on
  private double validTarget;
  private double x;
  private double y;
  private double area;
  private double skew;
  private double latency;
  private double shortest;
  private double longest;
  private double width;
  private double height;
  private int pipeIndex;

  // The possible camtran values
  /** the xPose of the robot in camtran */
  private double poseX;
  /** same but for y */
  private double poseY;
  /** same but for z */
  private double poseZ;
  /** up-down angle of the robot in camtran. Look it up if confused */
  private double pitch;
  /** side-to-side angle. Look it up if confused */
  private double yaw;
  /** rotation angle. Look it up if confused */
  private double roll;

  /**
   * Default constructor
   *
   * @param driverPipeline the pipeline for the driver camera
   */
  public Limelight(int driverPipeline) {
    this.driverPipeline = driverPipeline;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // Entries to get data
    validTargetTable = table.getEntry("tv");
    xTable = table.getEntry("tx");
    yTable = table.getEntry("ty");
    areaTable = table.getEntry("ta");
    skewTable = table.getEntry("ts");
    latencyTable = table.getEntry("tl");
    shortTable = table.getEntry("tshort");
    longTable = table.getEntry("tlong");
    widthTable = table.getEntry("thor");
    heightTable = table.getEntry("tvert");
    pipeTable = table.getEntry("getpipe");
    camtran = table.getEntry("camtran");
    cTable = table.getEntry("tc");
    // Entries to set data
    ledModeSet = table.getEntry("ledMode");
    camModeSet = table.getEntry("camMode");
    pipelineSet = table.getEntry("pipeline");
    streamSet = table.getEntry("stream");
    snapshotSet = table.getEntry("snapshot");

    setPipeline(driverPipeline);
  }

  @Override
  public void periodic() {
    pipeIndex = (int) pipeTable.getDouble(driverPipeline);
    validTarget = validTargetTable.getDouble(-1);
    if (pipeIndex != driverPipeline) {
      x = xTable.getDouble(0);
      y = yTable.getDouble(0);
      // System.out.println("X = " + x + ", y = " + y);
      area = areaTable.getDouble(0);
      skew = skewTable.getDouble(0);
      latency = latencyTable.getDouble(0);
      shortest = shortTable.getDouble(0);
      longest = longTable.getDouble(0);
      width = widthTable.getDouble(0);
      height = heightTable.getDouble(0);
      var camtranVals = camtran.getNumberArray(new Number[6]);
      poseX = camtranVals[0].doubleValue();
      poseY = camtranVals[1].doubleValue();
      poseZ = camtranVals[2].doubleValue();
      pitch = camtranVals[3].doubleValue();
      yaw = camtranVals[4].doubleValue();
      roll = camtranVals[5].doubleValue();
    }
  }

  @Log
  public boolean hasTarget() {
    return validTarget == 1;
  }

  @Log
  public double getX() {
    return x;
  }

  @Log
  public double getY() {
    return y;
  }

  @Log
  public double getArea() {
    return area;
  }

  @Log
  public double getSkew() {
    return skew;
  }

  @Log
  public double getLatency() {
    return latency;
  }

  @Log
  public double getShortest() {
    return shortest;
  }

  @Log
  public double getLongest() {
    return longest;
  }

  @Log
  public double getWidth() {
    return width;
  }

  @Log
  public double getHeight() {
    return height;
  }

  @Log
  public double getPipeline() {
    return pipeIndex;
  }

  @Log
  public void setPipeline(int index) {
    pipelineSet.setNumber(index);
  }

  @Log
  public double getPoseX() {
    return poseX;
  }

  @Log
  public double getPoseY() {
    return poseY;
  }

  @Log
  public double getPoseZ() {
    return poseZ;
  }

  @Log
  public double getPitch() {
    return pitch;
  }

  @Log
  public double getYaw() {
    return yaw;
  }

  @Log
  public double getRoll() {
    return roll;
  }

  public void setLedMode(@NotNull LedMode ledMode) {
    ledModeSet.setNumber(ledMode.asNum);
  }

  public void setStreamMode(@NotNull StreamMode streamMode) {
    streamSet.setNumber(streamMode.asNum);
  }

  /**
   * Set the 'snapshot' entry
   *
   * @param takeSnapshots {@code true} to take 2 snapshots per second, {@code false} to turn them
   *     off
   */
  public void setSnapshot(boolean takeSnapshots) {
    snapshotSet.setNumber(takeSnapshots ? 1 : 0);
  }

  public enum LedMode {
    /** Use the mode set for the current pipeline */
    CURRENT(0),
    OFF(1),
    BLINK(2),
    ON(3);

    public final Number asNum;

    LedMode(Number asNum) {
      this.asNum = asNum;
    }
  }

  /** Streaming mode for Limelight */
  public enum StreamMode {
    /** Side-by-side streams if a webcam is attached to Limelight */
    STANDARD(0),
    /**
     * PiP Main - The secondary camera stream is placed in the lower-right corner of the primary
     * camera stream
     */
    PIP_MAIN(1),
    /**
     * PiP Secondary - The primary camera stream is placed in the lower-right corner of the
     * secondary camera stream
     */
    PIP_SECONDARY(2);

    public final Number asNum;

    StreamMode(Number asNum) {
      this.asNum = asNum;
    }
  }
}
