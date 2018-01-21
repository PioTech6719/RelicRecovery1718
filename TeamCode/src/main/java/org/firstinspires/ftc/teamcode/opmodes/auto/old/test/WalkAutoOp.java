package org.firstinspires.ftc.teamcode.opmodes.auto.old.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.robots.old.Pionizer;

@Autonomous(name = "ESAD RUN THIS")
@Disabled
public class WalkAutoOp extends LinearOpMode {

    private Pionizer robot = new Pionizer();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAuto(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robot.getMoveProvider().moveRunToPosition(this, Directions.NORTH, 10);
            stop();
        }
    }
}
