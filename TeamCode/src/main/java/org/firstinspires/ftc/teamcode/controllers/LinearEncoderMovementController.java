package org.firstinspires.ftc.teamcode.controllers;

import android.support.annotation.NonNull;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.Prometheus;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class LinearEncoderMovementController {

    private Prometheus robot = null;

    public LinearEncoderMovementController(@NonNull Prometheus robot) {
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