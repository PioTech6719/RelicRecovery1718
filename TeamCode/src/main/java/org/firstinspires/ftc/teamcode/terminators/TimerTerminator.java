package org.firstinspires.ftc.teamcode.terminators;

public class TimerTerminator implements Terminator {

    private double start;
    private double seconds;

    public TimerTerminator(double seconds) {
        start = -1;
        this.seconds = seconds;
        //Init start now or in shouldTerminate
    }

    @Override
    public boolean shouldTerminate() {
        if (start == -1 && seconds > 0) {
            start = System.currentTimeMillis();
        }
        return System.currentTimeMillis() >= start + seconds;
    }
}
