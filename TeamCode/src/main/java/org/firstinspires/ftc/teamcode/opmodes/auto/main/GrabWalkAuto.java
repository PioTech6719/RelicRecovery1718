package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Directions;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.AutoTransitioner;

public class GrabWalkAuto extends LinearOpMode {

    private Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Path:", "Close arms, 20 inches to Safe Zone, Open Arms, Back 4 inches");
        robot.init(hardwareMap);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        AutoTransitioner.transitionOnStop(this, "Main TeleOp");
        waitForStart();

        robot.closeGlyphArms();
        robot.move(this, Directions.NORTH, 20);
        robot.openGlyphArms();
        robot.move(this, Directions.SOUTH, 4);

        telemetry.addData("Status:", "Done with Auto");
        telemetry.addData("Status:", "Transitioning to TeleOp");
        telemetry.update();

        stop();
    }
}
