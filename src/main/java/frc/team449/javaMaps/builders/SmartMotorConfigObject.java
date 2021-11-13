package frc.team449.javaMaps.builders;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.team449.components.RunningLinRegComponent;
import frc.team449.generalInterfaces.SmartMotor;
import frc.team449.generalInterfaces.shiftable.Shiftable;
import frc.team449.jacksonWrappers.PDP;
import frc.team449.jacksonWrappers.SlaveSparkMax;
import frc.team449.jacksonWrappers.SlaveTalon;
import frc.team449.jacksonWrappers.SlaveVictor;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/**
* The constructor for {@link SmartMotor} was hell so this will help resolve that.
* <p>
* You can set config options in this and then pass this to the {@link SmartMotor} constructor.
* <p>
* It's very similar to {@link SmartMotorBuilder} but has getters.
*
 * @see SmartMotor
 * @author Katie Del Toro
*/
public class SmartMotorConfigObject {
    private SmartMotor.Type type;
    private int port;
    private boolean enableBrakeMode;
    private String name;
    private boolean reverseOutput;
    private PDP pdp;
    private Boolean fwdLimitSwitchNormallyOpen;
    private Boolean revLimitSwitchNormallyOpen;
    private Integer remoteLimitSwitchID;
    private Double fwdSoftLimit;
    private Double revSoftLimit;
    private Double postEncoderGearing;
    private Double unitPerRotation;
    private Integer currentLimit;
    private boolean enableVoltageComp;
    private List<Shiftable.PerGearSettings> perGearSettings = new ArrayList<>();
    private Shiftable.Gear startingGear;
    private Integer startingGearNum;
    private Integer controlFrameRateMillis;
    private Map<ControlFrame, Integer> controlFrameRatesMillis = new EnumMap<>(ControlFrame.class);
    private RunningLinRegComponent voltagePerCurrentLinReg;
    private Integer voltageCompSamples;
    private FeedbackDevice feedbackDevice;
    private Integer encoderCPR;
    private Boolean reverseSensor;
    private Double updaterProcessPeriodSecs;
    private List<SlaveTalon> slaveTalons = new ArrayList<>();
    private List<SlaveVictor> slaveVictors = new ArrayList<>();
    private List<SlaveSparkMax> slaveSparks = new ArrayList<>();
    private Map<?, Integer> statusFrameRatesMillis;

    public SmartMotor.Type getType() {
        return type;
    }

    public SmartMotorConfigObject setType(SmartMotor.Type type) {
        this.type = type;
        return this;
    }

    public int getPort() {
        return port;
    }

    public SmartMotorConfigObject setPort(int port) {
        this.port = port;
        return this;
    }

    public boolean isEnableBrakeMode() {
        return enableBrakeMode;
    }

    public SmartMotorConfigObject setEnableBrakeMode(boolean enableBrakeMode) {
        this.enableBrakeMode = enableBrakeMode;
        return this;
    }

    public String getName() {
        return name;
    }

    public SmartMotorConfigObject setName(String name) {
        this.name = name;
        return this;
    }

    public boolean isReverseOutput() {
        return reverseOutput;
    }

    public SmartMotorConfigObject setReverseOutput(boolean reverseOutput) {
        this.reverseOutput = reverseOutput;
        return this;
    }

    public PDP getPdp() {
        return pdp;
    }

    public SmartMotorConfigObject setPdp(PDP pdp) {
        this.pdp = pdp;
        return this;
    }

    public Boolean getFwdLimitSwitchNormallyOpen() {
        return fwdLimitSwitchNormallyOpen;
    }

    public SmartMotorConfigObject setFwdLimitSwitchNormallyOpen(Boolean fwdLimitSwitchNormallyOpen) {
        this.fwdLimitSwitchNormallyOpen = fwdLimitSwitchNormallyOpen;
        return this;
    }

    public Boolean getRevLimitSwitchNormallyOpen() {
        return revLimitSwitchNormallyOpen;
    }

    public SmartMotorConfigObject setRevLimitSwitchNormallyOpen(Boolean revLimitSwitchNormallyOpen) {
        this.revLimitSwitchNormallyOpen = revLimitSwitchNormallyOpen;
        return this;
    }

    public Integer getRemoteLimitSwitchID() {
        return remoteLimitSwitchID;
    }

    public SmartMotorConfigObject setRemoteLimitSwitchID(Integer remoteLimitSwitchID) {
        this.remoteLimitSwitchID = remoteLimitSwitchID;
        return this;
    }

    public Double getFwdSoftLimit() {
        return fwdSoftLimit;
    }

    public SmartMotorConfigObject setFwdSoftLimit(Double fwdSoftLimit) {
        this.fwdSoftLimit = fwdSoftLimit;
        return this;
    }

    public Double getRevSoftLimit() {
        return revSoftLimit;
    }

    public SmartMotorConfigObject setRevSoftLimit(Double revSoftLimit) {
        this.revSoftLimit = revSoftLimit;
        return this;
    }

    public Double getPostEncoderGearing() {
        return postEncoderGearing;
    }

    public SmartMotorConfigObject setPostEncoderGearing(Double postEncoderGearing) {
        this.postEncoderGearing = postEncoderGearing;
        return this;
    }

    public Double getUnitPerRotation() {
        return unitPerRotation;
    }

    public SmartMotorConfigObject setUnitPerRotation(Double unitPerRotation) {
        this.unitPerRotation = unitPerRotation;
        return this;
    }

    public Integer getCurrentLimit() {
        return currentLimit;
    }

    public SmartMotorConfigObject setCurrentLimit(Integer currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean isEnableVoltageComp() {
        return enableVoltageComp;
    }

    public SmartMotorConfigObject setEnableVoltageComp(boolean enableVoltageComp) {
        this.enableVoltageComp = enableVoltageComp;
        return this;
    }

    public List<Shiftable.PerGearSettings> getPerGearSettings() {
        return perGearSettings;
    }

    public SmartMotorConfigObject setPerGearSettings(List<Shiftable.PerGearSettings> perGearSettings) {
        this.perGearSettings = perGearSettings;
        return this;
    }

    public Shiftable.Gear getStartingGear() {
        return startingGear;
    }

    public SmartMotorConfigObject setStartingGear(Shiftable.Gear startingGear) {
        this.startingGear = startingGear;
        return this;
    }

    public Integer getStartingGearNum() {
        return startingGearNum;
    }

    public SmartMotorConfigObject setStartingGearNum(Integer startingGearNum) {
        this.startingGearNum = startingGearNum;
        return this;
    }

    public Integer getControlFrameRateMillis() {
        return controlFrameRateMillis;
    }

    public SmartMotorConfigObject setControlFrameRateMillis(Integer controlFrameRateMillis) {
        this.controlFrameRateMillis = controlFrameRateMillis;
        return this;
    }

    public Map<ControlFrame, Integer> getControlFrameRatesMillis() {
        return controlFrameRatesMillis;
    }

    public SmartMotorConfigObject setControlFrameRatesMillis(Map<ControlFrame, Integer> controlFrameRatesMillis) {
        this.controlFrameRatesMillis = controlFrameRatesMillis;
        return this;
    }

    public RunningLinRegComponent getVoltagePerCurrentLinReg() {
        return voltagePerCurrentLinReg;
    }

    public SmartMotorConfigObject setVoltagePerCurrentLinReg(RunningLinRegComponent voltagePerCurrentLinReg) {
        this.voltagePerCurrentLinReg = voltagePerCurrentLinReg;
        return this;
    }

    public Integer getVoltageCompSamples() {
        return voltageCompSamples;
    }

    public SmartMotorConfigObject setVoltageCompSamples(Integer voltageCompSamples) {
        this.voltageCompSamples = voltageCompSamples;
        return this;
    }

    public FeedbackDevice getFeedbackDevice() {
        return feedbackDevice;
    }

    public SmartMotorConfigObject setFeedbackDevice(FeedbackDevice feedbackDevice) {
        this.feedbackDevice = feedbackDevice;
        return this;
    }

    public Integer getEncoderCPR() {
        return encoderCPR;
    }

    public SmartMotorConfigObject setEncoderCPR(Integer encoderCPR) {
        this.encoderCPR = encoderCPR;
        return this;
    }

    public Boolean getReverseSensor() {
        return reverseSensor;
    }

    public SmartMotorConfigObject setReverseSensor(Boolean reverseSensor) {
        this.reverseSensor = reverseSensor;
        return this;
    }

    public Double getUpdaterProcessPeriodSecs() {
        return updaterProcessPeriodSecs;
    }

    public SmartMotorConfigObject setUpdaterProcessPeriodSecs(Double updaterProcessPeriodSecs) {
        this.updaterProcessPeriodSecs = updaterProcessPeriodSecs;
        return this;
    }

    public List<SlaveTalon> getSlaveTalons() {
        return slaveTalons;
    }

    public SmartMotorConfigObject setSlaveTalons(List<SlaveTalon> slaveTalons) {
        this.slaveTalons = slaveTalons;
        return this;
    }

    public List<SlaveVictor> getSlaveVictors() {
        return slaveVictors;
    }

    public SmartMotorConfigObject setSlaveVictors(List<SlaveVictor> slaveVictors) {
        this.slaveVictors = slaveVictors;
        return this;
    }

    public List<SlaveSparkMax> getSlaveSparks() {
        return slaveSparks;
    }

    public SmartMotorConfigObject setSlaveSparks(List<SlaveSparkMax> slaveSparks) {
        this.slaveSparks = slaveSparks;
        return this;
    }

    public Map<?, Integer> getStatusFrameRatesMillis() {
        return statusFrameRatesMillis;
    }

    public SmartMotorConfigObject setStatusFrameRatesMillis(Map<?, Integer> statusFrameRatesMillis) {
        this.statusFrameRatesMillis = statusFrameRatesMillis;
        return this;
    }
}
