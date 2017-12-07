package org.firstinspires.ftc.teamcode.opmodes.tele.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.Constants;

@TeleOp(name = "Sensor - Drive Encoders", group = "test")
public class EncoderPushbotTest extends LinearOpMode {

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    public void initialize() {
        leftFrontMotor = (DcMotor) hardwareMap.get("leftFront");
        rightFrontMotor = (DcMotor) hardwareMap.get("rightFront");
        leftRearMotor = (DcMotor) hardwareMap.get("leftRear");
        rightRearMotor = (DcMotor) hardwareMap.get("rightRear");

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


        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            } else {
                leftFrontMotor.setPower(0);
                rightFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightRearMotor.setPower(0);
            }

            if (gamepad1.right_trigger > Constants.GAMEPAD_THRESHOLD) {
                leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            telemetry.addData("Left Front: ", leftFrontMotor.getCurrentPosition());
            telemetry.addData("Right Front: ", rightFrontMotor.getCurrentPosition());
            telemetry.addData("Left Rear: ", leftRearMotor.getCurrentPosition());
            telemetry.addData("Right Rear: ", rightRearMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
