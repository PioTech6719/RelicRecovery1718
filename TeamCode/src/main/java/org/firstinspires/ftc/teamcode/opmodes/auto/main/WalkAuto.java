package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Directions;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.AutoTransitioner;

@Autonomous(name = "Walk Auto", group = "auto")
public class WalkAuto extends LinearOpMode {

    private Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Path:", "15 North, 5 South");
        telemetry.update();

        robot.init(hardwareMap);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        AutoTransitioner.transitionOnStop(this, "Robot TeleOp 1");
        waitForStart();

        robot.move(this, Directions.NORTH, 15);

        robot.move(-0.5, -0.5, this, Directions.SOUTH, 3, 5);

        telemetry.addData("Status:", "Done with Auto");
        telemetry.addData("Status:", "Transitioning to TeleOp");
        telemetry.update();

        stop();
    }
}
