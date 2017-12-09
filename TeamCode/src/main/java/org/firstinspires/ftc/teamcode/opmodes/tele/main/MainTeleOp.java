package org.firstinspires.ftc.teamcode.opmodes.tele.main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.robots.Pionizer;
import org.firstinspires.ftc.teamcode.utils.Constants;

@TeleOp(name = "Prometheus TeleOp 1", group = "main")
public class MainTeleOp extends OpMode{

    private Pionizer robot = new Pionizer();
    private double defaultLinearSpeed = 1.0;

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
        //Linear + Rotational Movement
        if(gamepad1.dpad_up) { //North

            robot.frontLeft.setPower(defaultLinearSpeed);
            robot.frontRight.setPower(defaultLinearSpeed);
            robot.backLeft.setPower(defaultLinearSpeed);
            robot.backRight.setPower(defaultLinearSpeed);

        } else if (gamepad1.dpad_down) {

            robot.frontLeft.setPower(-defaultLinearSpeed);
            robot.frontRight.setPower(-defaultLinearSpeed);
            robot.backLeft.setPower(-defaultLinearSpeed);
            robot.backRight.setPower(-defaultLinearSpeed);

        } else if (gamepad1.dpad_left){

            robot.frontLeft.setPower(-defaultLinearSpeed);
            robot.frontRight.setPower(defaultLinearSpeed);
            robot.backLeft.setPower(defaultLinearSpeed);
            robot.backRight.setPower(-defaultLinearSpeed);

        } else if (gamepad1.dpad_right) {

            robot.frontLeft.setPower(defaultLinearSpeed);
            robot.frontRight.setPower(-defaultLinearSpeed);
            robot.backLeft.setPower(-defaultLinearSpeed);
            robot.backRight.setPower(defaultLinearSpeed);

        } else if (gamepad1.left_trigger > Constants.GAMEPAD_THRESHOLD){ //Rotate Left

            robot.frontLeft.setPower(-0.4);
            robot.frontRight.setPower(0.4);
            robot.backLeft.setPower(-0.4);
            robot.backRight.setPower(0.4);

        } else if (gamepad1.right_trigger > Constants.GAMEPAD_THRESHOLD) { //Rotate Right

            robot.frontLeft.setPower(0.4);
            robot.frontRight.setPower(-0.4);
            robot.backLeft.setPower(0.4);
            robot.backRight.setPower(-0.4);

        } else {

            robot.frontLeft.setPower(0.0);
            robot.frontRight.setPower(0.0);
            robot.backLeft.setPower(0.0);
            robot.backRight.setPower(0.0);

        }

        //Speed Settings
        if (gamepad1.a) {
            defaultLinearSpeed = 0.75;
        }
        if (gamepad1.b) {
            defaultLinearSpeed = 0.5;
        }
        if (gamepad1.y) {
            defaultLinearSpeed = 1.0;
        }

        if (gamepad2.dpad_up){
            robot.ballArmUp();
        } else if (gamepad2.dpad_down) {
            robot.ballArmDown();
        } else if (gamepad2.a) {
            robot.ballArmReset();
        }

        //Glyph Arm Control
        if(gamepad2.left_bumper){
            robot.closeGlyphArms();
        }
        if (gamepad2.right_bumper) {
            robot.openGlyphArms();
        }

        //Manual Lift Control
        if (-gamepad2.left_stick_y > Constants.GAMEPAD_THRESHOLD) {
            robot.liftMotor.setPower(1.0);
        } else if (-gamepad2.left_stick_y < -Constants.GAMEPAD_THRESHOLD) {
            robot.liftMotor.setPower(-1.0);
        } else{
            robot.liftMotor.setPower(0);
        }

        //Encoder Lift Control
        if (gamepad2.y) {
            Pionizer.LiftSetting desiredLiftSetting = Pionizer.LiftSetting.HIGH;
            if (robot.currentLiftSetting == Pionizer.LiftSetting.LOW) {
                desiredLiftSetting = Pionizer.LiftSetting.MID;
            } else if (robot.currentLiftSetting == Pionizer.LiftSetting.MID) {
                desiredLiftSetting = Pionizer.LiftSetting.HIGH;
            }
            robot.setLiftSystem(desiredLiftSetting);
        } else if (gamepad2.a) {
            Pionizer.LiftSetting desiredLiftSetting = Pionizer.LiftSetting.LOW;
            if (robot.currentLiftSetting == Pionizer.LiftSetting.MID) {
                desiredLiftSetting = Pionizer.LiftSetting.LOW;
            } else if (robot.currentLiftSetting == Pionizer.LiftSetting.HIGH) {
                desiredLiftSetting = Pionizer.LiftSetting.MID;
            }
            robot.setLiftSystem(desiredLiftSetting);
        }
}}
