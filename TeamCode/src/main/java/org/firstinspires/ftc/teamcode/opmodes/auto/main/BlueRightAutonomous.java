package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Directions;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Blue Right w/ Color", group = "auto")
public class BlueRightAutonomous extends LinearOpMode {
    private Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
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

        robot.move(this, Directions.WEST, 20);
        robot.imuRotate(180, Directions.ROTATE_CCW, 5);

        robot.openGlyphArms();
        robot.move(this, Directions.NORTH, 20);

        robot.move(this, Directions.SOUTH, 5);
        stop();
    }
}
