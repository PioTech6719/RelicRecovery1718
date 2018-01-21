package org.firstinspires.ftc.teamcode.opmodes.auto.old.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.robots.old.Pionizer;


@Disabled
public class EncoderTest extends LinearOpMode {

    private Pionizer robot = new Pionizer();
    private double distance = 10 / 3;
    private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAuto(hardwareMap);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {
            //G1
            if (gamepad1.a) {
                robot.getMoveProvider().moveSimplePID(this, runMode, Directions.NORTH, distance);
            }

            if (gamepad1.b) {
                robot.getMoveProvider().moveSimplePID(this, runMode, Directions.SOUTH, distance);
            }

            if (gamepad1.x) {
                robot.getMoveProvider().moveSimplePID(this, runMode, Directions.EAST, distance);
            }

            if (gamepad1.y) {
                robot.getMoveProvider().moveSimplePID(this, runMode, Directions.WEST, distance);
            }

            //G2
            if (gamepad2.a) {
                robot.getMoveProvider().moveRunToPosition(this, Directions.NORTH, (int) distance);
            }

            if (gamepad2.b) {
                robot.getMoveProvider().moveRunToPosition(this, Directions.SOUTH, (int) distance);
            }

            if (gamepad2.x) {
                robot.getMoveProvider().moveRunToPosition(this, Directions.EAST, (int) distance);
            }

            if (gamepad2.y) {
                robot.getMoveProvider().moveRunToPosition(this, Directions.WEST, (int) distance);
            }

            if (gamepad1.left_bumper) {
                distance = 15 / 3;
            }

            if (gamepad1.right_bumper) {
                distance = 20 / 3;
            }

            if (gamepad1.left_trigger > 0.4) {
                runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            }

            if (gamepad1.right_trigger > 0.4) {
                runMode = DcMotor.RunMode.RUN_USING_ENCODER;
            }
        }
    }

}
