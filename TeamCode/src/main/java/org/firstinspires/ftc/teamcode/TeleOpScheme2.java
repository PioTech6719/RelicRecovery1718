package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Constants;

@TeleOp(name = "Robot TeleOp 2", group = "test")
public class TeleOpScheme2 extends OpMode {

    private Robot robot = new Robot();
    private final double GAMEPAD_THRESHOLD = 0.2;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        robot.init(hardwareMap);
        gamepad1.setJoystickDeadzone(0.1f);
        gamepad2.setJoystickDeadzone(0.1f);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void loop() {
        //Linear Movement
        if (gamepad1.left_trigger > GAMEPAD_THRESHOLD) {
            robot.driveXY(gamepad1.left_stick_x * Constants.SLOW_COEFFICIENT, -gamepad1.left_stick_y * Constants.SLOW_COEFFICIENT);
        } else {
            robot.driveXY(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        }

        //Rotational Movement
        if (gamepad1.left_trigger > GAMEPAD_THRESHOLD) {
            robot.rotateXY(gamepad1.right_stick_x * Constants.SLOW_COEFFICIENT);
        } else {
            robot.rotateXY(gamepad1.right_stick_x);
        }

        if (gamepad2.dpad_up) {
            robot.ballArmUp();
        } else if (gamepad2.dpad_down) {
            robot.ballArmDown();
        } else if (gamepad2.a) {
            robot.ballArmReset();
        } else if (gamepad2.b) {
            robot.ballPush.setPosition(0.493);
        } else if (gamepad2.left_bumper) {
            robot.arm1.setPosition(30);
            robot.arm2.setPosition(0);
        } else if (gamepad2.right_bumper) {
            robot.arm1.setPosition(-30);
            robot.arm2.setPosition(60);
        } else if (-gamepad2.left_stick_y > 0.4) {
            robot.liftMotor.setPower(1.0);
        } else if (-gamepad2.left_stick_y < -0.4) {
            robot.liftMotor.setPower(-1.0);
        } else {
            robot.liftMotor.setPower(0);
        }
    }
}
