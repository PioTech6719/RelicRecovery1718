package org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID;

public class PIDBuilder {

    private double KP;
    private double KI;
    private double KD;
    private double KF;
    private double TOLERANCE;
    private double SETTLING_TIME;
    private double TARGET_MIN_RANGE;
    private double TARGET_MAX_RANGE;
    private double MIN_OUTPUT;
    private double MAX_OUTPUT;
    private double TARGET;
    private boolean INVERTED, ABSOLUTE_SETPOINT, NO_OSCILLATION;

    public double getKP() {
        return KP;
    }

    public PIDBuilder setKP(double KP) {
        this.KP = KP;
        return this;
    }

    public double getKI() {
        return KI;
    }

    public PIDBuilder setKI(double KI) {
        this.KI = KI;
        return this;
    }

    public double getKD() {
        return KD;
    }

    public PIDBuilder setKD(double KD) {
        this.KD = KD;
        return this;
    }

    public double getKF() {
        return KF;
    }

    public PIDBuilder setKF(double KF) {
        this.KF = KF;
        return this;
    }

    public double getTOLERANCE() {
        return TOLERANCE;
    }

    public PIDBuilder setTOLERANCE(double TOLERANCE) {
        this.TOLERANCE = TOLERANCE;
        return this;
    }

    public double getSETTLING_TIME() {
        return SETTLING_TIME;
    }

    public PIDBuilder setSETTLING_TIME(double SETTLING_TIME) {
        this.SETTLING_TIME = SETTLING_TIME;
        return this;
    }

    public double getTARGET_MIN_RANGE() {
        return TARGET_MIN_RANGE;
    }

    public double getTARGET_MAX_RANGE() {
        return TARGET_MAX_RANGE;
    }

    public double getMIN_OUTPUT() {
        return MIN_OUTPUT;
    }

    public double getMAX_OUTPUT() {
        return MAX_OUTPUT;
    }

    public boolean isINVERTED() {
        return INVERTED;
    }

    public PIDBuilder setINVERTED(boolean INVERTED) {
        this.INVERTED = INVERTED;
        return this;
    }

    public boolean isABSOLUTE_SETPOINT() {
        return ABSOLUTE_SETPOINT;
    }

    public PIDBuilder setABSOLUTE_SETPOINT(boolean ABSOLUTE_SETPOINT) {
        this.ABSOLUTE_SETPOINT = ABSOLUTE_SETPOINT;
        return this;
    }

    public boolean isNO_OSCILLATION() {
        return NO_OSCILLATION;
    }

    public PIDBuilder setNO_OSCILLATION(boolean NO_OSCILLATION) {
        this.NO_OSCILLATION = NO_OSCILLATION;
        return this;
    }

    public PIDBuilder setTARGET_RANGE(double TARGET_MIN_RANGE, double TARGET_MAX_RANGE) {
        this.TARGET_MIN_RANGE = TARGET_MIN_RANGE;
        this.TARGET_MAX_RANGE = TARGET_MAX_RANGE;
        return this;
    }

    public PIDBuilder setOUTPUT_RANGE(double MIN_OUTPUT, double MAX_OUTPUT) {
        this.MIN_OUTPUT = MIN_OUTPUT;
        this.MAX_OUTPUT = MAX_OUTPUT;
        return this;
    }

    public PIDBuilder setTARGET(double TARGET) {
        this.TARGET = TARGET;
        return this;
    }
}
