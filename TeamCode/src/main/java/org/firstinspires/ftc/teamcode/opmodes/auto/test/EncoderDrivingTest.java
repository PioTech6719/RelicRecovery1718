package org.firstinspires.ftc.teamcode.opmodes.auto.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;

@Autonomous(name = "Sensor - Encoder Driving Test", group = "test")
public class EncoderDrivingTest extends LinearOpMode {

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    public void initialize() {
        leftFrontMotor = (DcMotor) hardwareMap.get(PrometheusConstants.LEFT_FRONT_DRIVE_MOTOR);
        rightFrontMotor = (DcMotor) hardwareMap.get(PrometheusConstants.RIGHT_FRONT_DRIVE_MOTOR);
        leftRearMotor = (DcMotor) hardwareMap.get(PrometheusConstants.LEFT_REAR_DRIVE_MOTOR);
        rightRearMotor = (DcMotor) hardwareMap.get(PrometheusConstants.RIGHT_REAR_DRIVE_MOTOR);

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.idle();

        telemetry.addData("Left Front: ", leftFrontMotor.getCurrentPosition());
        telemetry.addData("Right Front: ", rightFrontMotor.getCurrentPosition());
        telemetry.addData("Left Rear: ", leftRearMotor.getCurrentPosition());
        telemetry.addData("Right Rear: ", rightRearMotor.getCurrentPosition());
        telemetry.update();


        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gamepad1.setJoystickDeadzone(.1f);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                leftFrontMotor.setPower(1);
                rightFrontMotor.setPower(1);
                leftRearMotor.setPower(1);
                rightRearMotor.setPower(1);
            } else if (gamepad1.dpad_down) {
                leftFrontMotor.setPower(-1);
                rightFrontMotor.setPower(-1);
                leftRearMotor.setPower(-1);
                rightRearMotor.setPower(-1);
            } else if (gamepad1.dpad_right) {
                leftFrontMotor.setPower(1);
                rightFrontMotor.setPower(-1);
                leftRearMotor.setPower(-1);
                rightRearMotor.setPower(1);
            } else if (gamepad1.dpad_left) {
                leftFrontMotor.setPower(-1);
                rightFrontMotor.setPower(1);
                leftRearMotor.setPower(1);
                rightRearMotor.setPower(-1);
            } else if (gamepad1.x) {
                leftFrontMotor.setPower(-1);
                rightFrontMotor.setPower(1);
                leftRearMotor.setPower(-1);
                rightRearMotor.setPower(1);
            } else if (gamepad1.b) {
                leftFrontMotor.setPower(1);
                rightFrontMotor.setPower(-1);
                leftRearMotor.setPower(1);
                rightRearMotor.setPower(-1);
            } else {
                leftFrontMotor.setPower(0);
                rightFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightRearMotor.setPower(0);
            }

            if (gamepad1.right_bumper) {
                leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("Run Mode: ", "Run Without Encoder");
            }

            if (gamepad1.left_bumper) {
                leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Run Mode: ", "Run Using Encoder");
            }

            telemetry.addData("Left Front: ", leftFrontMotor.getCurrentPosition());
            telemetry.addData("Right Front: ", rightFrontMotor.getCurrentPosition());
            telemetry.addData("Left Rear: ", leftRearMotor.getCurrentPosition());
            telemetry.addData("Right Rear: ", rightRearMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
