package org.firstinspires.ftc.teamcode.opmodes.auto.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoTransitioner;

@Autonomous(name = "Auto Transitioner Test", group = "test")
public class AutoTransitionerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutoTransitioner.transitionOnStop(this, "Prometheus TeleOp 1");
        waitForStart();

        while (opModeIsActive()) {
            stop();
        }
    }
}
