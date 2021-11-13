package frc.team449;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team449.javaMaps.Bunnybot2021Map;
import frc.team449.other.Clock;
import io.github.oblarg.oblog.Logger;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.*;

/** The main class of the robot, constructs all the subsystems and initializes default commands. */
public class Robot extends TimedRobot {
  private static boolean isUnitTesting = false;
  private static boolean isTestingHasBeenCalled = false;
  /** The object constructed directly from the yaml map. */
  @NotNull protected final RobotMap robotMap = Objects.requireNonNull(loadMap());

  /** The method that runs when the robot is turned on. Initializes all subsystems from the map. */
  public static @Nullable RobotMap loadMap() {
    return Bunnybot2021Map.createRobotMap();
  }

  /**
   * Whether robot code is being unit tested. Note that this is NOT the same as test mode.
   *
   * <p>The return value will never change observably. {@link Robot#notifyTesting()} will thus throw
   * an exception if it is called after the first time that this method is called.
   *
   * @return whether the current run is a unit test
   */
  public static boolean isUnitTesting() {
    isTestingHasBeenCalled = true;
    return isUnitTesting;
  }

  /**
   * Notifies robot code that it is being unit tested.
   *
   * @throws UnsupportedOperationException if the robot is not running in a simulation
   * @throws IllegalStateException if {@link Robot#isUnitTesting()} has already been called before
   *     this method is called
   */
  public static void notifyTesting() throws UnsupportedOperationException, IllegalStateException {
    if (RobotBase.isReal())
      throw new IllegalStateException(
          "Attempt to enable unit testing mode while not running in simulation");

    if (isUnitTesting) return;
    if (isTestingHasBeenCalled)
      throw new IllegalStateException("isTesting() has already been called at least once");

    System.out.println("ROBOT UNIT TESTING");
    isUnitTesting = true;
  }

  @Override
  public void robotInit() {
    // Set up start time
    Clock.setStartTime();

    // Yes this should be a print statement, it's useful to know that robotInit started.
    System.out.println("Started robotInit.");

    if (this.robotMap.useCameraServer()) {
      CameraServer.getInstance().startAutomaticCapture();
    }

    // Read sensors
    this.robotMap.getUpdater().run();

    Logger.configureLoggingAndConfig(this.robotMap, false);
    Shuffleboard.setRecordingFileNameFormat("log-${time}");
    Shuffleboard.startRecording();

    // start systems
    if (this.robotMap.getRobotStartupCommands() != null) {
      this.robotMap.getRobotStartupCommands().forEachRemaining(Command::schedule);
    }
  }

  @Override
  public void robotPeriodic() {
    // save current time
    Clock.updateTime();
    // Read sensors
    this.robotMap.getUpdater().run();
    // update shuffleboard
    Logger.updateEntries();
    // Run all commands. This is a WPILib thing you don't really have to worry about.
    CommandScheduler.getInstance().run();
  }

  /** Run when we first enable in teleop. */
  @Override
  public void teleopInit() {
    // cancel remaining auto commands
    if (this.robotMap.getAutoStartupCommands() != null) {
      this.robotMap.getAutoStartupCommands().forEachRemaining(Command::cancel);
    }

    // Run teleop startup commands
    if (this.robotMap.getTeleopStartupCommands() != null) {
      this.robotMap.getTeleopStartupCommands().forEachRemaining(Command::schedule);
    }
  }

  /** Run when we first enable in autonomous */
  @Override
  public void autonomousInit() {
    // Run the auto startup command
    if (this.robotMap.getAutoStartupCommands() != null) {
      this.robotMap.getAutoStartupCommands().forEachRemaining(Command::schedule);
    }
  }

  /** Run when we first enable in test mode. */
  @Override
  public void testInit() {
    // Run startup command if we start in test mode.
    if (this.robotMap.getTestStartupCommands() != null) {
      this.robotMap.getTestStartupCommands().forEachRemaining(Command::schedule);
    }
  }

  /** Formatting for map reference chain of exception caused by map error. */
  private enum MapErrorFormat {
    /** The chain is printed as-is on one line. */
    NONE,
    /** The chain is split up into one frame per line and left-justified. */
    LEFT_ALIGN,
    /** The chain is split up into one frame per line and right-justified. */
    RIGHT_ALIGN,
    /**
     * The chain is split up into one frame per line and formatted as a table with locations to the
     * left of class names.
     */
    TABLE
  }
}
