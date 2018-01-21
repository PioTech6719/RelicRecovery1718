package org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.consts;

public class CustomPIDConstants implements PIDConstants {

    private double KP = 0;
    private double KI = 0;
    private double KD = 0;
    private double KF = 0;
    private double TOLERANCE = 0;
    private double SETTLING_TIME = 0;
    private double TARGET_MIN_RANGE;
    private double TARGET_MAX_RANGE;
    private double MIN_OUTPUT = 0;
    private double MAX_OUTPUT = 0;
    private double TARGET = 0;
    private boolean INVERTED, ABSOLUTE_SETPOINT, NO_OSCILLATION = false;

    @Override
    public double getKP() {
        return KP;
    }

    public CustomPIDConstants setKP(double KP) {
        this.KP = KP;
        return this;
    }

    @Override
    public double getKD() {
        return KD;
    }

    public CustomPIDConstants setKD(double KD) {
        this.KD = KD;
        return this;
    }

    @Override
    public double getKI() {
        return KI;
    }

    public CustomPIDConstants setKI(double KI) {
        this.KI = KI;
        return this;
    }

    @Override
    public double getKF() {
        return KF;
    }

    public CustomPIDConstants setKF(double KF) {
        this.KF = KF;
        return this;
    }

    @Override
    public double getTOLERANCE() {
        return TOLERANCE;
    }

    public CustomPIDConstants setTOLERANCE(double TOLERANCE) {
        this.TOLERANCE = TOLERANCE;
        return this;
    }

    @Override
    public double getSETTLING_TIME() {
        return SETTLING_TIME;
    }

    public CustomPIDConstants setSETTLING_TIME(double SETTLING_TIME) {
        this.SETTLING_TIME = SETTLING_TIME;
        return this;
    }

    @Override
    public double getMIN_OUTPUT() {
        return MIN_OUTPUT;
    }

    public CustomPIDConstants setMIN_OUTPUT(double MIN_OUTPUT) {
        this.MIN_OUTPUT = MIN_OUTPUT;
        return this;
    }

    @Override
    public double getMAX_OUTPUT() {
        return MAX_OUTPUT;
    }

    public CustomPIDConstants setMAX_OUTPUT(double MAX_OUTPUT) {
        this.MAX_OUTPUT = MAX_OUTPUT;
        return this;
    }

    public double getTARGET_MIN_RANGE() {
        return TARGET_MIN_RANGE;
    }

    public CustomPIDConstants setTARGET_MIN_RANGE(double TARGET_MIN_RANGE) {
        this.TARGET_MIN_RANGE = TARGET_MIN_RANGE;
        return this;
    }

    public double getTARGET_MAX_RANGE() {
        return TARGET_MAX_RANGE;
    }

    public CustomPIDConstants setTARGET_MAX_RANGE(double TARGET_MAX_RANGE) {
        this.TARGET_MAX_RANGE = TARGET_MAX_RANGE;
        return this;
    }

    @Override
    public double getTARGET() {
        return TARGET;
    }

    public CustomPIDConstants setTARGET(double TARGET) {
        this.TARGET = TARGET;
        return this;
    }

    @Override
    public boolean isINVERTED() {
        return INVERTED;
    }

    public CustomPIDConstants setINVERTED(boolean INVERTED) {
        this.INVERTED = INVERTED;
        return this;
    }

    @Override
    public boolean isABSOLUTE_SETPOINT() {
        return ABSOLUTE_SETPOINT;
    }

    public CustomPIDConstants setABSOLUTE_SETPOINT(boolean ABSOLUTE_SETPOINT) {
        this.ABSOLUTE_SETPOINT = ABSOLUTE_SETPOINT;
        return this;
    }

    @Override
    public boolean isNO_OSCILLATION() {
        return NO_OSCILLATION;
    }

    public CustomPIDConstants setNO_OSCILLATION(boolean NO_OSCILLATION) {
        this.NO_OSCILLATION = NO_OSCILLATION;
        return this;
    }
}
