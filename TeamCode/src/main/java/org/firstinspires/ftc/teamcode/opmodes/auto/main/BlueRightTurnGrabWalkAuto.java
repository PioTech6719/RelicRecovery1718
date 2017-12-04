package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.Prometheus;

@Autonomous(name = "Blue Right Turn Grab Walk", group = "auto")
public class BlueRightTurnGrabWalkAuto extends LinearOpMode {

    private Prometheus robot = new Prometheus();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Path:", "Close arms, 20 inches north, 90 CCW, Open Arms, 20 inches north, Back 5 inches");
        telemetry.update();

        robot.init(hardwareMap);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        waitForStart();

        robot.closeGlyphArms();
        //Raise lift or no?

        robot.move(this, Directions.NORTH, 20);
        robot.imuRotate(90, Directions.ROTATE_CCW, 5);
        robot.openGlyphArms();
        robot.move(this, Directions.NORTH, 20);
        robot.move(this, Directions.SOUTH, 5);

        telemetry.addData("Status:", "Done with Auto");
//        telemetry.addData("Status:", "Transitioning to TeleOp");
        telemetry.update();

        stop();
    }
}
