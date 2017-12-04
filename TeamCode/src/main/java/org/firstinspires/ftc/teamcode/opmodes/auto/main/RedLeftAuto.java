package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.Prometheus;

@Autonomous(name = "Red Left w/ Color", group = "auto")
public class RedLeftAuto extends LinearOpMode {

    private Prometheus robot = new Prometheus();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Orient:", "Back facing jewels w/ glyph in middle of arms");
        telemetry.addData("Path:", "Close arms, 4 South, Knock Ball, 5 North, 13 East, 12 North, " +
                "Rotate to 90 CW, Open arms, 20 North, 5 South");

        robot.init(hardwareMap);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

//        AutoTransitioner.transitionOnStop(this, "Main TeleOp");
        waitForStart();

        robot.closeGlyphArms();
        robot.move(this, Directions.SOUTH, 4);
        robot.ballArmDown();
        sleep(1500);

        if (robot.getColor() == Color.BLUE) {
            robot.imuRotate(15, Directions.ROTATE_CCW, 3);
            robot.ballArmUp();
            sleep(1500);
            robot.imuRotate(-15, Directions.ROTATE_CW, 3);
        } else {
            robot.imuRotate(15, Directions.ROTATE_CW, 3);
            robot.ballArmUp();
            sleep(1500);
            robot.imuRotate(-15, Directions.ROTATE_CCW, 3);
        }

        robot.move(this, Directions.NORTH, 5);

        robot.move(this, Directions.EAST, 13);
        robot.move(this, Directions.NORTH, 12);
        robot.imuRotateTo(90, Directions.ROTATE_CW, 5); //does this work or rotate? or simple rotateto?

        robot.openGlyphArms();

        robot.move(this, Directions.NORTH, 20);
        robot.move(this, Directions.SOUTH, 5);

        robot.ballArmReset();

        stop();
    }

}
