package frc.team449.generalInterfaces;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.generalInterfaces.shiftable.Shiftable;
import frc.team449.generalInterfaces.simpleMotor.SimpleMotor;
import frc.team449.jacksonWrappers.*;
import frc.team449.jacksonWrappers.simulated.MPSSmartMotorSimulated;
import frc.team449.javaMaps.builders.SmartMotorConfigObject;
import frc.team449.other.Updater;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import static frc.team449.other.Util.getLogPrefix;

/**
 * A motor with built-in advanced capability featuring encoder, current limiting, and gear shifting
 * support. Also features built in MPS conversions.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public interface SmartMotor extends SimpleMotor, Shiftable, Loggable {
  /**
   * Whether to construct instances of {@link MPSSmartMotorSimulated} instead of the specified
   * controllers when the robot is running in a simulation.
   */
  boolean SIMULATE = true;
  /**
   * Whether to simulate sparks if they cause a HAL error when constructed.
   */
  boolean SIMULATE_SPARKS_IF_ERR = true;

  int LOG_WIDTH = 4, LOG_HEIGHT = 3; //TODO Unused

  /**
   * Creates a new <b>SPARK</b> or <b>Talon</b> motor controller.
   * @param config A {@link SmartMotorConfigObject} containing the configuration info.
   */
  @JsonCreator
  static SmartMotor create(SmartMotorConfigObject config) {
    SmartMotor.Type type = config.getType();
    int port = config.getPort();
    boolean enableBrakeMode = config.isEnableBrakeMode();
    String name = config.getName();
    boolean reverseOutput = config.isReverseOutput();
    PDP pdp = config.getPdp();
    Boolean fwdLimitSwitchNormallyOpen = config.getFwdLimitSwitchNormallyOpen();
    Boolean revLimitSwitchNormallyOpen = config.getRevLimitSwitchNormallyOpen();
    Integer remoteLimitSwitchID = config.getRemoteLimitSwitchID();
    Double fwdSoftLimit = config.getFwdSoftLimit();
    Double revSoftLimit = config.getRevSoftLimit();
    Double postEncoderGearing = config.getPostEncoderGearing();
    Double unitPerRotation = config.getUnitPerRotation();
    Integer currentLimit = config.getCurrentLimit();
    boolean enableVoltageComp = config.isEnableVoltageComp();
    List<Shiftable.PerGearSettings> perGearSettings = config.getPerGearSettings();
    Shiftable.Gear startingGear = config.getStartingGear();
    Integer startingGearNum = config.getStartingGearNum();
    Integer controlFrameRateMillis = config.getControlFrameRateMillis();
    Map<ControlFrame, Integer> controlFrameRatesMillis = config.getControlFrameRatesMillis();
    RunningLinRegComponent voltagePerCurrentLinReg = config.getVoltagePerCurrentLinReg();
    Integer voltageCompSamples = config.getVoltageCompSamples();
    FeedbackDevice feedbackDevice = config.getFeedbackDevice();
    Integer encoderCPR = config.getEncoderCPR();
    Boolean reverseSensor = config.getReverseSensor();
    Double updaterProcessPeriodSecs = config.getUpdaterProcessPeriodSecs();
    List<SlaveTalon> slaveTalons = config.getSlaveTalons();
    List<SlaveVictor> slaveVictors = config.getSlaveVictors();
    List<SlaveSparkMax> slaveSparks = config.getSlaveSparks();
    Map<?, Integer> statusFrameRatesMillis = config.getStatusFrameRatesMillis();
    final var logHelper =
            new Object() {
              public void warning(final String message) {
                this.log("Warning: " + message);
              }

              public void log(final String message) {
                this.direct("       " + message);
              }

              public void direct(final String message) {
                System.out.print(getLogPrefix(SmartMotor.class));
                System.out.println(message);
              }

              public void error(final String message) {
                this.log("ERROR: " + message);
              }
            };

    final String motorLogName = String.format("%s \"%s\" on port %d", type, name, port);

    logHelper.direct("Constructing " + motorLogName);
    final Type actualType;

    if (SIMULATE && RobotBase.isSimulation()) {
      actualType = Type.SIMULATED;
    } else if (SIMULATE_SPARKS_IF_ERR && type == Type.SPARK) {
      try (final var spark = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushless)) {
        spark.restoreFactoryDefaults();
        if (spark.getLastError() == CANError.kHALError) {
          actualType = Type.SIMULATED;
          logHelper.warning(
                  "error for spark on port "
                          + port
                          + "; assuming nonexistent and replacing with simulated controller");
        } else {
          actualType = type;
        }
      }
    } else {
      actualType = type;
    }

    final var unsupportedHelper =
            new Object() {
              public void log(final String property) {
                logHelper.warning("Property " + property + " is not supported for " + actualType);
              }
            };

    // The status frame map must be dealt with manually because Jackson gives the frames as raw
    // strings due to the
    // type parameter being a wildcard (Object). The solution is to invoke Jackson again to parse
    // them.
    final var sparkStatusFramesMap = new EnumMap<CANSparkMaxLowLevel.PeriodicFrame, Integer>(CANSparkMaxLowLevel.PeriodicFrame.class);
    final var talonStatusFramesMap = new EnumMap<StatusFrameEnhanced, Integer>(StatusFrameEnhanced.class);

    if (statusFrameRatesMillis != null) {
      for (final Object frame : statusFrameRatesMillis.entrySet()) {
        if (frame instanceof String) {
          // Must put it in quotes so Jackson recognizes it as a string.
          final String toBeParsed = "\"" + frame + "\"";
          try {
            if (actualType == Type.TALON) {
              talonStatusFramesMap.put(
                      new ObjectMapper().readValue(toBeParsed, StatusFrameEnhanced.class),
                      statusFrameRatesMillis.get(frame));
            } else if (actualType == Type.SPARK) {
              sparkStatusFramesMap.put(
                      new ObjectMapper().readValue(toBeParsed, CANSparkMaxLowLevel.PeriodicFrame.class),
                      statusFrameRatesMillis.get(frame));
            }
          } catch (final Exception ex) {
            logHelper.error(" Could not parse status frame rate key value " + toBeParsed);
            throw new RuntimeException(ex);
          }

        } else if (frame instanceof CANSparkMaxLowLevel.PeriodicFrame) {
          if (type == Type.TALON)
            throw new IllegalArgumentException(
                    "statusFrameRatesMillis contains key of type CANSparkMaxLowLevel.PeriodicFrame that will not work for MPSTalon");
          sparkStatusFramesMap.put(
                  (CANSparkMaxLowLevel.PeriodicFrame) frame, statusFrameRatesMillis.get(frame));

        } else if (frame instanceof StatusFrameEnhanced) {
          if (actualType == Type.SPARK)
            throw new IllegalArgumentException(
                    "statusFrameRatesMillis contains key of type StatusFrameEnhanced that will not work for MPSSparkMax");
          talonStatusFramesMap.put((StatusFrameEnhanced) frame, statusFrameRatesMillis.get(frame));

        } else {
          throw new IllegalArgumentException(
                  "statusFrameRatesMillis contains key of unexpected type "
                          + frame.getClass().getName());
        }
      }
    }

    final SmartMotor result;
    switch (actualType) {
      case SPARK:
        if (slaveTalons != null) unsupportedHelper.log("slaveTalons");
        if (slaveVictors != null) unsupportedHelper.log("slaveTalons");
        if (voltagePerCurrentLinReg != null) unsupportedHelper.log("voltagePerCurrentLinReg");
        if (encoderCPR != null) unsupportedHelper.log("encoderCPR");
        if (reverseSensor != null) unsupportedHelper.log("reverseSensor");
        if (voltageCompSamples != null) unsupportedHelper.log("voltageCompSamples");
        if (updaterProcessPeriodSecs != null) unsupportedHelper.log("updaterProcessPeriodSecs");
        if (controlFrameRatesMillis != null)
          unsupportedHelper.log("controlFrameRatesMillis (RATESSSS--plural)");

        result =
                new MappedSparkMax(
                        port,
                        name,
                        reverseOutput,
                        enableBrakeMode,
                        pdp,
                        fwdLimitSwitchNormallyOpen,
                        revLimitSwitchNormallyOpen,
                        remoteLimitSwitchID,
                        fwdSoftLimit,
                        revSoftLimit,
                        postEncoderGearing,
                        unitPerRotation,
                        currentLimit,
                        enableVoltageComp,
                        perGearSettings,
                        startingGear,
                        startingGearNum,
                        sparkStatusFramesMap,
                        controlFrameRateMillis,
                        slaveSparks);
        break;

      case TALON:
        if (controlFrameRateMillis != null)
          unsupportedHelper.log("controlFrameRatesMillis (RATE--singular)");

        result =
                new MappedTalon(
                        port,
                        name,
                        reverseOutput,
                        enableBrakeMode,
                        voltagePerCurrentLinReg,
                        pdp,
                        fwdLimitSwitchNormallyOpen,
                        revLimitSwitchNormallyOpen,
                        remoteLimitSwitchID,
                        fwdSoftLimit,
                        revSoftLimit,
                        postEncoderGearing,
                        unitPerRotation,
                        currentLimit,
                        enableVoltageComp,
                        voltageCompSamples,
                        feedbackDevice,
                        encoderCPR,
                        reverseSensor != null ? reverseSensor : false,
                        perGearSettings,
                        startingGear,
                        startingGearNum,
                        talonStatusFramesMap,
                        controlFrameRatesMillis,
                        slaveTalons,
                        slaveVictors,
                        slaveSparks);
        break;

      case SIMULATED:
        logHelper.log("SIM:  " + motorLogName);
        final var simulated =
                new MPSSmartMotorSimulated(
                        actualType,
                        port,
                        enableBrakeMode,
                        name,
                        reverseOutput,
                        pdp,
                        fwdLimitSwitchNormallyOpen,
                        revLimitSwitchNormallyOpen,
                        remoteLimitSwitchID,
                        fwdSoftLimit,
                        revSoftLimit,
                        postEncoderGearing,
                        unitPerRotation,
                        currentLimit,
                        enableVoltageComp,
                        perGearSettings,
                        startingGear,
                        startingGearNum,
                        sparkStatusFramesMap,
                        controlFrameRateMillis,
                        talonStatusFramesMap,
                        controlFrameRatesMillis,
                        voltagePerCurrentLinReg,
                        voltageCompSamples,
                        feedbackDevice,
                        encoderCPR,
                        reverseSensor,
                        updaterProcessPeriodSecs,
                        slaveTalons,
                        slaveVictors,
                        slaveSparks);
        Updater.subscribe(simulated);
        result = simulated;
        break;

      default:
        throw new IllegalArgumentException("Unsupported motor type: " + actualType);
    }

    logHelper.direct("SUCCESS:     " + motorLogName);

    MotorContainer.register(result);
    return result;
  }

  /**
   * Old constructor, don't use this
   * @deprecated since {@link SmartMotor#create(SmartMotorConfigObject)}
   */
  @Deprecated(since = "other constructor was added", forRemoval = true)
  static SmartMotor create(
          SmartMotor.Type type,
          int port,
          boolean enableBrakeMode,
          String name,
          boolean reverseOutput,
          PDP pdp,
          Boolean fwdLimitSwitchNormallyOpen,
          Boolean revLimitSwitchNormallyOpen,
          Integer remoteLimitSwitchID,
          Double fwdSoftLimit,
          Double revSoftLimit,
          Double postEncoderGearing,
          Double unitPerRotation,
          Integer currentLimit,
          boolean enableVoltageComp,
          List<Shiftable.PerGearSettings> perGearSettings,
          Shiftable.Gear startingGear,
          Integer startingGearNum,
          Integer controlFrameRateMillis,
          Map<ControlFrame, Integer> controlFrameRatesMillis,
          RunningLinRegComponent voltagePerCurrentLinReg,
          Integer voltageCompSamples,
          FeedbackDevice feedbackDevice,
          Integer encoderCPR,
          Boolean reverseSensor,
          Double updaterProcessPeriodSecs,
          List<SlaveTalon> slaveTalons,
          List<SlaveVictor> slaveVictors,
          List<SlaveSparkMax> slaveSparks,
          Map<?, Integer> statusFrameRatesMillis) {
    final var logHelper =
            new Object() {
              public void warning(final String message) {
                this.log("Warning: " + message);
              }

              public void log(final String message) {
                this.direct("       " + message);
              }

              public void direct(final String message) {
                System.out.print(getLogPrefix(SmartMotor.class));
                System.out.println(message);
              }

              public void error(final String message) {
                this.log("ERROR: " + message);
              }
            };

    final String motorLogName = String.format("%s \"%s\" on port %d", type, name, port);

    logHelper.direct("Constructing " + motorLogName);

    final Type actualType;

    if (SIMULATE && RobotBase.isSimulation()) {
      actualType = Type.SIMULATED;
    } else if (SIMULATE_SPARKS_IF_ERR && type == Type.SPARK) {
      try (final var spark = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushless)) {
        spark.restoreFactoryDefaults();
        if (spark.getLastError() == CANError.kHALError) {
          actualType = Type.SIMULATED;
          logHelper.warning(
                  "error for spark on port "
                          + port
                          + "; assuming nonexistent and replacing with simulated controller");
        } else {
          actualType = type;
        }
      }
    } else {
      actualType = type;
    }

    final var unsupportedHelper =
            new Object() {
              public void log(final String property) {
                logHelper.warning("Property " + property + " is not supported for " + actualType);
              }
            };

    // The status frame map must be dealt with manually because Jackson gives the frames as raw
    // strings due to the
    // type parameter being a wildcard (Object). The solution is to invoke Jackson again to parse
    // them.
    final var sparkStatusFramesMap = new EnumMap<CANSparkMaxLowLevel.PeriodicFrame, Integer>(CANSparkMaxLowLevel.PeriodicFrame.class);
    final var talonStatusFramesMap = new EnumMap<StatusFrameEnhanced, Integer>(StatusFrameEnhanced.class);

    if (statusFrameRatesMillis != null) {
      for (final Object frame : statusFrameRatesMillis.entrySet()) {
        if (frame instanceof String) {
          // Must put it in quotes so Jackson recognizes it as a string.
          final String toBeParsed = "\"" + frame + "\"";
          try {
            if (actualType == Type.TALON) {
              talonStatusFramesMap.put(
                      new ObjectMapper().readValue(toBeParsed, StatusFrameEnhanced.class),
                      statusFrameRatesMillis.get(frame));
            } else if (actualType == Type.SPARK) {
              sparkStatusFramesMap.put(
                      new ObjectMapper().readValue(toBeParsed, CANSparkMaxLowLevel.PeriodicFrame.class),
                      statusFrameRatesMillis.get(frame));
            }
          } catch (final Exception ex) {
            logHelper.error(" Could not parse status frame rate key value " + toBeParsed);
            throw new RuntimeException(ex);
          }

        } else if (frame instanceof CANSparkMaxLowLevel.PeriodicFrame) {
          if (type == Type.TALON)
            throw new IllegalArgumentException(
                    "statusFrameRatesMillis contains key of type CANSparkMaxLowLevel.PeriodicFrame that will not work for MPSTalon");
          sparkStatusFramesMap.put(
                  (CANSparkMaxLowLevel.PeriodicFrame) frame, statusFrameRatesMillis.get(frame));

        } else if (frame instanceof StatusFrameEnhanced) {
          if (actualType == Type.SPARK)
            throw new IllegalArgumentException(
                    "statusFrameRatesMillis contains key of type StatusFrameEnhanced that will not work for MPSSparkMax");
          talonStatusFramesMap.put((StatusFrameEnhanced) frame, statusFrameRatesMillis.get(frame));

        } else {
          throw new IllegalArgumentException(
                  "statusFrameRatesMillis contains key of unexpected type "
                          + frame.getClass().getName());
        }
      }
    }

    final SmartMotor result;

    switch (actualType) {
      case SPARK:
        if (slaveTalons != null) unsupportedHelper.log("slaveTalons");
        if (slaveVictors != null) unsupportedHelper.log("slaveTalons");
        if (voltagePerCurrentLinReg != null) unsupportedHelper.log("voltagePerCurrentLinReg");
        if (encoderCPR != null) unsupportedHelper.log("encoderCPR");
        if (reverseSensor != null) unsupportedHelper.log("reverseSensor");
        if (voltageCompSamples != null) unsupportedHelper.log("voltageCompSamples");
        if (updaterProcessPeriodSecs != null) unsupportedHelper.log("updaterProcessPeriodSecs");
        if (controlFrameRatesMillis != null)
          unsupportedHelper.log("controlFrameRatesMillis (RATESSSS--plural)");

        result =
                new MappedSparkMax(
                        port,
                        name,
                        reverseOutput,
                        enableBrakeMode,
                        pdp,
                        fwdLimitSwitchNormallyOpen,
                        revLimitSwitchNormallyOpen,
                        remoteLimitSwitchID,
                        fwdSoftLimit,
                        revSoftLimit,
                        postEncoderGearing,
                        unitPerRotation,
                        currentLimit,
                        enableVoltageComp,
                        perGearSettings,
                        startingGear,
                        startingGearNum,
                        sparkStatusFramesMap,
                        controlFrameRateMillis,
                        slaveSparks);
        break;

      case TALON:
        if (controlFrameRateMillis != null)
          unsupportedHelper.log("controlFrameRatesMillis (RATE--singular)");

        result =
                new MappedTalon(
                        port,
                        name,
                        reverseOutput,
                        enableBrakeMode,
                        voltagePerCurrentLinReg,
                        pdp,
                        fwdLimitSwitchNormallyOpen,
                        revLimitSwitchNormallyOpen,
                        remoteLimitSwitchID,
                        fwdSoftLimit,
                        revSoftLimit,
                        postEncoderGearing,
                        unitPerRotation,
                        currentLimit,
                        enableVoltageComp,
                        voltageCompSamples,
                        feedbackDevice,
                        encoderCPR,
                        reverseSensor != null ? reverseSensor : false,
                        perGearSettings,
                        startingGear,
                        startingGearNum,
                        talonStatusFramesMap,
                        controlFrameRatesMillis,
                        slaveTalons,
                        slaveVictors,
                        slaveSparks);
        break;

      case SIMULATED:
        logHelper.log("SIM:  " + motorLogName);
        final var simulated =
                new MPSSmartMotorSimulated(
                        actualType,
                        port,
                        enableBrakeMode,
                        name,
                        reverseOutput,
                        pdp,
                        fwdLimitSwitchNormallyOpen,
                        revLimitSwitchNormallyOpen,
                        remoteLimitSwitchID,
                        fwdSoftLimit,
                        revSoftLimit,
                        postEncoderGearing,
                        unitPerRotation,
                        currentLimit,
                        enableVoltageComp,
                        perGearSettings,
                        startingGear,
                        startingGearNum,
                        sparkStatusFramesMap,
                        controlFrameRateMillis,
                        talonStatusFramesMap,
                        controlFrameRatesMillis,
                        voltagePerCurrentLinReg,
                        voltageCompSamples,
                        feedbackDevice,
                        encoderCPR,
                        reverseSensor,
                        updaterProcessPeriodSecs,
                        slaveTalons,
                        slaveVictors,
                        slaveSparks);
        Updater.subscribe(simulated);
        result = simulated;
        break;

      default:
        throw new IllegalArgumentException("Unsupported motor type: " + actualType);
    }

    logHelper.direct("SUCCESS:     " + motorLogName);

    MotorContainer.register(result);
    return result;
  }

  /**
   * Set the motor output voltage to a given percent of available voltage.
   * @param percentVoltage percent of total voltage from [-1, 1]
   */
  void setPercentVoltage(double percentVoltage);

  /**
   * Convert from native units read by an encoder to meters moved. Note this DOES account for
   * post-encoder gearing.
   * @param nativeUnits A distance native units as measured by the encoder.
   * @return That distance in meters, or null if no encoder CPR was given.
   */
  double encoderToUnit(double nativeUnits);

  /**
   * Convert a distance from meters to encoder reading in native units. Note this DOES account for
   * post-encoder gearing.
   * @param meters A distance in meters.
   * @return That distance in native units as measured by the encoder, or null if no encoder CPR was
   * given.
   */
  double unitToEncoder(double meters);

  /**
   * Converts the velocity read by the controller's getVelocity() method to the MPS of the output
   * shaft. Note this DOES account for post-encoder gearing.
   * @param encoderReading The velocity read from the encoder with no conversions.
   * @return The velocity of the output shaft, in MPS, when the encoder has that reading, or null if
   * no encoder CPR was given.
   */
  double encoderToUPS(double encoderReading);

  /**
   * Converts from the velocity of the output shaft to what the controller's getVelocity() method
   * would read at that velocity. Note this DOES account for post-encoder gearing.
   * @param mps The velocity of the output shaft, in MPS.
   * @return What the raw encoder reading would be at that velocity, or null if no encoder CPR was
   * given.
   */
  double upsToEncoder(double mps);

  /**
   * Convert from native velocity units to output rotations per second. Note this DOES NOT account
   * for post-encoder gearing.
   * @param nat A velocity in native units.
   * @return That velocity in RPS, or null if no encoder CPR was given.
   */
  Double nativeToRPS(double nat);

  /**
   * Convert from output RPS to the native velocity. Note this DOES NOT account for post-encoder
   * gearing.
   * @param rps The RPS velocity you want to convert.
   * @return That velocity in native units, or null if no encoder CPR was given.
   */
  double rpsToNative(double rps);

  /**
   * @return Raw position units for debugging purposes
   */
  double encoderPosition();

  /**
   * Set a position setpoint for the controller.
   */
  void setPositionSetpoint(double meters);

  /**
   * @return Raw velocity units for debugging purposes
   */
  double encoderVelocity();

  /**
   * Sets the output in volts.
   */
  void setVoltage(double volts);

  /**
   * Get the velocity of the controller in MPS.
   * @return The controller's velocity in MPS, or null if no encoder CPR was given.
   */
  double getVelocity();

  /**
   * Set the velocity for the motor to go at.
   * @param velocity the desired velocity, on [-1, 1].
   */
  @Override
  void setVelocity(double velocity);

  /**
   * Give a velocity closed loop setpoint in MPS.
   * @param velocity velocity setpoint in MPS.
   */
  void setVelocityUPS(double velocity);

  /**
   * Get the current closed-loop velocity error in MPS. WARNING: will give garbage if not in
   * velocity mode.
   * @return The closed-loop error in MPS, or null if no encoder CPR was given.
   */
  double getError(); //TODO Unused

  /**
   * Get the current velocity setpoint of the motor in MPS, the position setpoint in meters
   * @return The setpoint in sensible units for the current control mode.
   */
  double getSetpoint();

  /**
   * Get the voltage the motor is currently drawing from the PDP.
   * @return Voltage in volts.
   */
  double getOutputVoltage();

  /**
   * Get the voltage available for the motor.
   * @return Voltage in volts.
   */
  double getBatteryVoltage();

  /**
   * Get the current the motor is currently drawing from the PDP.
   * @return Current in amps.
   */
  double getOutputCurrent();

  /**
   * Get the current control mode of the motor. Please don't use this for anything other than
   * logging.
   * @return Control mode as a string.
   */
  String getControlMode(); //TODO Unused

  /**
   * Set the velocity scaled to a given gear's max velocity. Used mostly when autoshifting.
   * @param velocity The velocity to go at, from [-1, 1], where 1 is the max speed of the given
   *                 gear.
   * @param gear     The number of the gear to use the max speed from to scale the velocity.
   */
  void setGearScaledVelocity(double velocity, int gear);

  /**
   * Set the velocity scaled to a given gear's max velocity. Used mostly when autoshifting.
   * @param velocity The velocity to go at, from [-1, 1], where 1 is the max speed of the given
   *                 gear.
   * @param gear     The gear to use the max speed from to scale the velocity.
   */
  void setGearScaledVelocity(double velocity, Gear gear);

  /**
   * @return Feedforward calculator for this gear
   */
  SimpleMotorFeedforward getCurrentGearFeedForward();

  /**
   * @return the position of the motor in meters, or null of inches per rotation wasn't given.
   */
  double getPositionUnits();

  /**
   * Resets the position of the motor to 0.
   */
  void resetPosition();

  /**
   * Get the status of the forwards limit switch.
   * @return True if the forwards limit switch is closed, false if it's open or doesn't exist.
   */
  boolean getFwdLimitSwitch(); //TODO Unused

  /**
   * Get the status of the reverse limit switch.
   * @return True if the reverse limit switch is closed, false if it's open or doesn't exist.
   */
  boolean getRevLimitSwitch(); //TODO Unused

  boolean isInhibitedForward();

  boolean isInhibitedReverse();

  /**
   * Gets the CAN port of this controller.
   * @return the CAN port of this controller
   */
  int getPort();

  /**
   * Gets the name of this instance of the class.
   * @return the name of this instance when logging
   */
  @Override
  String configureLogName();

  @Override
  default LayoutType configureLayoutType() {
    return BuiltInLayouts.kGrid;
  }

  /**
   * Gets the default width and height of the layout of this instance of the class in Shuffleboard.
   * f
   * @return an array of {width, height}.
   */
  @Override
  default int[] configureLayoutSize() {
    return new int[] {4, 3};
  }

  @Override
  default int[] configureLayoutPosition() {
    return new int[] {3, 3};
  }

  /**
   * Gets whether the motor is a simulated motor.
   * @return whether the motor is a software simulation of a motor
   */
  @Log
  default boolean isSimulated() { //TODO Unused
    return false;
  }

  enum Type {
    /**
     * RevRobotics SPARK MAX
     */
    SPARK("SparkMax"),
    /**
     * CTRE Talon SRX
     */
    TALON("Talon"),
    /**
     * Simulated motor
     * @see MPSSmartMotorSimulated
     */
    SIMULATED("SIMULATED");

    public final String friendlyName;

    Type(final String friendlyName) {
      this.friendlyName = friendlyName;
    }

    @Override
    public String toString() {
      return this.friendlyName;
    }
  }
}
