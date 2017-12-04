package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.Prometheus;

@Autonomous(name = "Blue Right w/ Color", group = "auto")
public class BlueRightAuto extends LinearOpMode {

    private Prometheus robot = new Prometheus();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Path:", "Close arms, 4 South, Arm Down, Knock Ball, 5 North, 20 West, " +
                "Rotate 180 CCW, Open arms, 20 North, 5 South");
        telemetry.update();

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

        robot.move(this, Directions.WEST, 20);
        robot.imuRotate(180, Directions.ROTATE_CCW, 5); //rotateto or does this work??

        robot.openGlyphArms();
        robot.move(this, Directions.NORTH, 20);

        robot.move(this, Directions.SOUTH, 5);

        robot.ballArmReset();

        stop();
    }
}
