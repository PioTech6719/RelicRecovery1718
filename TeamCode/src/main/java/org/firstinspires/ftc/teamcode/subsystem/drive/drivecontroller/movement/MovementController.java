package org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.movement;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PIDComplexity;
import org.firstinspires.ftc.teamcode.terminators.Terminator;
import org.firstinspires.ftc.teamcode.utils.PioTimer;

public interface MovementController {

    void move(Directions direction, double distance, PioTimer timer, PIDComplexity pidComplexity);

    void move(Directions direction, double distance, double driveSpeed, PioTimer timer, PIDComplexity pidComplexity); //Replace w/ combined distance and time terminator (ask for them in the parameters) later

    void move(Directions direction, PioTimer timer, Terminator terminator); //Add PID Complexity later and cleaner system uncluding drivespeed wo diff method

    void move(Directions direction, double driveSpeed, PioTimer timer, Terminator terminator);

    void rotate(Directions direction, double angle, PioTimer timer, PIDComplexity pidComplexity);

    void rotate(Directions direction, double angle, double driveSpeed, PioTimer timer, PIDComplexity pidComplexity);

    void rotateTo(double targetAngle, PioTimer timer, PIDComplexity pidComplexity);

    void rotateTo(double targetAngle, double driveSpeed, PioTimer timer, PIDComplexity pidComplexity);
}
