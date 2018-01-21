package org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.consts;

public interface PIDConstants {

    double getKP();

    double getKI();

    double getKD();

    double getKF();

    double getTOLERANCE();

    double getSETTLING_TIME();

    double getTARGET_MIN_RANGE();

    double getTARGET_MAX_RANGE();

    double getMIN_OUTPUT();

    double getMAX_OUTPUT();

    double getTARGET();

    boolean isINVERTED();

    boolean isABSOLUTE_SETPOINT();

    boolean isNO_OSCILLATION();
}
