package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.Prometheus;

@Autonomous(name = "Straight Grab Walk", group = "auto")
public class StraightGrabWalkAuto extends LinearOpMode {

    private Prometheus robot = new Prometheus();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Path:", "Close arms, 20 inches to Safe Zone, Open Arms, Back 5 inches");
        telemetry.update();

        robot.init(hardwareMap);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

//        AutoTransitioner.transitionOnStop(this, "Main TeleOp");
        waitForStart();

        robot.closeGlyphArms();
        robot.move(this, Directions.NORTH, 15);
        robot.openGlyphArms();
        robot.move(this, Directions.SOUTH, 5);

        telemetry.addData("Status:", "Done with Auto");
//        telemetry.addData("Status:", "Transitioning to TeleOp");
        telemetry.update();

        stop(); //Remove if AutoTrans in use
    }
}
