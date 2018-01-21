package org.firstinspires.ftc.teamcode.terminators;

public class OpModeTerminator implements Terminator {

    @Override
    public boolean shouldTerminate() {
        return Status.isStopRequested();
    }
}
