package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.AutoTransitioner;

@Autonomous(name = "Auto w/ Color",group = "auto")
public class RedAutonomous extends LinearOpMode {
    private Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        AutoTransitioner.transitionOnStop(this, "Main TeleOp");
        waitForStart();


        robot.ballArmUp();
        if (robot.getColor()== Color.RED) {
            robot.move(this, Directions.ROTATE_LEFT, 10);
            robot.ballArmUp();
            robot.move(this, Directions.ROTATE_RIGHT, 10);
        } else {
            robot.move(this, Directions.ROTATE_LEFT, 5);
            robot.ballArmUp();
            robot.move(this, Directions.ROTATE_RIGHT, 5);
        }

        robot.move(this, Directions.NORTH, 20);
        stop();
    }
}
