package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PioTimer {

    private ElapsedTime timer = null;
    private ElapsedTime.Resolution resolution = null;

    private double duration = 0.0;
    private double tolerance = 0.0;
    private double startTime = 0.0;

    //TODO: Also add stop conditions like 7244
    public PioTimer(ElapsedTime.Resolution resolution, double duration, double tolerance) {
        timer = new ElapsedTime(resolution);

        this.resolution = resolution;
        this.duration = duration;

        if (tolerance == -1) {
            switch (resolution) {

                case SECONDS:
                    tolerance = Constants.TIMER_TOLERANCE_SECONDS;
                    break;
                case MILLISECONDS:
                    tolerance = Constants.TIMER_TOLERANCE_MILLISECONDS;
                    break;
            }
        }
    }

    public void start() {
        reset();
        startTime = timer.time();
    }

    private void reset() {
        timer.reset();
    }

    public boolean isFinished() {
        return timer.seconds() - (startTime + duration) >= tolerance;
    }
}
