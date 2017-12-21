package org.firstinspires.ftc.teamcode.subsystem.drive.movement.encoder;

import android.support.annotation.NonNull;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.robots.Pionizer;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class LinearEncoderMovementController {

    /**
     * TODO: make all these centralized and for specific robots so that once speicifcs are clicked
     * the same moves can be played. for example dpad up no matter joystick moves up
     */
    private Pionizer robot = null;

    public LinearEncoderMovementController(@NonNull Pionizer robot) {
        this.robot = robot;
    }

    public void moveDist(Directions direction, double distance, int timeoutSeconds) {
        if (timeoutSeconds == -1) {
            timeoutSeconds = Constants.DEFAULT_MOVE_TIMEOUT;
        } else if (timeoutSeconds == 0) {
            //ignore timeout - should rarely be used
        }


    }
}