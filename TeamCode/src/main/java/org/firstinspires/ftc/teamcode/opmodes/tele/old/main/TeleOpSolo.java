package org.firstinspires.ftc.teamcode.opmodes.tele.old.main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.robots.old.Pionizer;
import org.firstinspires.ftc.teamcode.utils.Constants;

@TeleOp(name = "Prometheus TeleOp Solo", group = "main")
public class TeleOpSolo extends OpMode {

    private Pionizer robot = new Pionizer();
    private double defaultLinearSpeed = 1.0;
    private boolean servosInit = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        robot.initTele(hardwareMap);
        gamepad1.setJoystickDeadzone(0.1f);
        gamepad2.setJoystickDeadzone(0.1f);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void loop() {
        if (!servosInit) {
            robot.initServos();
            servosInit = true;
        }

        //Linear + Rotational Movement
        if (gamepad1.dpad_up) { //North

            robot.frontLeft.setPower(defaultLinearSpeed);
            robot.frontRight.setPower(defaultLinearSpeed);
            robot.backLeft.setPower(defaultLinearSpeed);
            robot.backRight.setPower(defaultLinearSpeed);

        } else if (gamepad1.dpad_down) {

            robot.frontLeft.setPower(-defaultLinearSpeed);
            robot.frontRight.setPower(-defaultLinearSpeed);
            robot.backLeft.setPower(-defaultLinearSpeed);
            robot.backRight.setPower(-defaultLinearSpeed);

        } else if (gamepad1.dpad_left) {

            robot.frontLeft.setPower(-defaultLinearSpeed);
            robot.frontRight.setPower(defaultLinearSpeed);
            robot.backLeft.setPower(defaultLinearSpeed);
            robot.backRight.setPower(-defaultLinearSpeed);

        } else if (gamepad1.dpad_right) {

            robot.frontLeft.setPower(defaultLinearSpeed);
            robot.frontRight.setPower(-defaultLinearSpeed);
            robot.backLeft.setPower(-defaultLinearSpeed);
            robot.backRight.setPower(defaultLinearSpeed);

        } else if (gamepad1.x) { //Rotate Left

            robot.frontLeft.setPower(-0.5);
            robot.frontRight.setPower(0.5);
            robot.backLeft.setPower(-0.5);
            robot.backRight.setPower(0.5);

        } else if (gamepad1.b) { //Rotate Right

            robot.frontLeft.setPower(0.5);
            robot.frontRight.setPower(-0.5);
            robot.backLeft.setPower(0.5);
            robot.backRight.setPower(-0.5);

        } else {

            robot.frontLeft.setPower(0.0);
            robot.frontRight.setPower(0.0);
            robot.backLeft.setPower(0.0);
            robot.backRight.setPower(0.0);

        }

//        //Speed Settings
//        if (gamepad1.a) {
//            defaultLinearSpeed = 0.75;
//        }
//        if (gamepad1.b) {
//            defaultLinearSpeed = 0.5;
//        }
//        if (gamepad1.y) {
//            defaultLinearSpeed = 1.0;
//        }

//        //Jewel Arm Control
//        if (gamepad2.dpad_up){
//            robot.ballArmUp();
//        } else if (gamepad2.dpad_down) {
//            robot.ballArmDown();
//        } else if (gamepad2.a) {
//            robot.ballArmReset();
//        }

        //Glyph Arm Control
        if (gamepad1.left_bumper) {
            robot.closeUpperGlyphArms();
        }
        if (gamepad1.right_bumper) {
            robot.openUpperGlyphArms();
        }

        if (gamepad1.left_trigger > 0.4) {
            robot.closeLowerGlyphArms();
        }
        if (gamepad1.right_trigger > 0.4) {
            robot.openLowerGlyphArms();
        }

        //Manual Lift Control
        if (-gamepad1.left_stick_y > Constants.GAMEPAD_THRESHOLD) {
            robot.liftMotor.setPower(-0.7);
        } else if (-gamepad1.left_stick_y < -Constants.GAMEPAD_THRESHOLD) {
            robot.liftMotor.setPower(0.7);
        } else {
            robot.liftMotor.setPower(0);
        }
    }
}
