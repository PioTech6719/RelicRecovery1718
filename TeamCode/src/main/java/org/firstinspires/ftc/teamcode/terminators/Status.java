package org.firstinspires.ftc.teamcode.terminators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Used to tell if an OpMode is still running and acts as a terminator to stop all threads
 * Easier than sending around {@link com.qualcomm.robotcore.eventloop.opmode.OpMode} code
 */
public class Status {

    /**
     * Checks if the autonomous OpMode is requesting or has requested a stop
     *
     * @return if opMode is stopped
     */
    public static boolean isStopRequested() {
        if (Robot.getOpMode() instanceof LinearOpMode) {
            return ((LinearOpMode) Robot.getOpMode()).isStopRequested() || Thread.interrupted();
        }

        return false;
    }
}
