package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Directions;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Red Right Turn Grab Walk", group = "auto")
public class RedRightTurnGrabWalkAuto extends LinearOpMode {

    private Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Path:", "Close arms, 20 inches north, 90 CW, Open Arms, 20 inches north, Back 5 inches");
        robot.init(hardwareMap);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        waitForStart();

        robot.closeGlyphArms();
        //Raise lift or no?

        robot.move(this, Directions.NORTH, 20);
        robot.imuRotate(90, Directions.ROTATE_CW, 5);
        robot.openGlyphArms();
        robot.move(this, Directions.NORTH, 20);
        robot.move(this, Directions.SOUTH, 5);

        telemetry.addData("Status:", "Done with Auto");
//        telemetry.addData("Status:", "Transitioning to TeleOp");
        telemetry.update();

        stop();
    }
}
