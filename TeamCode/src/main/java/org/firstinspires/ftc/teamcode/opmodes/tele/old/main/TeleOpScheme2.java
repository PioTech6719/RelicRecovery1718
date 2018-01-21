package org.firstinspires.ftc.teamcode.opmodes.tele.old.main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.hardware.robots.old.Pionizer;
import org.firstinspires.ftc.teamcode.utils.Constants;

@TeleOp(name = "Prometheus TeleOp 2", group = "test")
public class TeleOpScheme2 extends OpMode {

    private final double GAMEPAD_THRESHOLD = 0.2;
    private Pionizer robot = new Pionizer();

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
        robot.move(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                gamepad1.right_stick_x, -gamepad1.right_stick_y);

        telemetry.addLine().addData("G1 Left Stick X : ", new Func<String>() {
            @Override
            public String value() {
                return Float.toString(gamepad1.left_stick_x);
            }
        });

        telemetry.addLine().addData("G1 Left Stick Y : ", new Func<String>() {
            @Override
            public String value() {
                return Float.toString(gamepad1.left_stick_y);
            }
        });

        telemetry.addLine().addData("G1 Right Stick X : ", new Func<String>() {
            @Override
            public String value() {
                return Float.toString(gamepad1.right_stick_x);
            }
        });

        telemetry.addLine().addData("G1 Right Stick Y : ", new Func<String>() {
            @Override
            public String value() {
                return Float.toString(gamepad1.right_stick_y);
            }
        });

        //Jewel Arm Control
        if (gamepad2.dpad_up) {
            robot.ballArmUp();
        } else if (gamepad2.dpad_down) {
            robot.ballArmDown();
        } else if (gamepad2.a) {
            robot.ballArmReset();
        }

        //Glyph Arm Control
        if (gamepad2.left_bumper) {
            robot.closeUpperGlyphArms();
        }
        if (gamepad2.right_bumper) {
            robot.openUpperGlyphArms();
        }

        if (gamepad2.left_trigger > 0.4) {
            robot.closeLowerGlyphArms();
        }
        if (gamepad2.right_trigger > 0.4) {
            robot.openLowerGlyphArms();
        }

        //Manual Lift Control
        if (-gamepad2.left_stick_y > Constants.GAMEPAD_THRESHOLD) {
            robot.liftMotor.setPower(-0.7);
        } else if (-gamepad2.left_stick_y < -Constants.GAMEPAD_THRESHOLD) {
            robot.liftMotor.setPower(0.7);
        } else {
            robot.liftMotor.setPower(0);
        }

        if (gamepad2.x) {
            robot.closeLowerGlyphArms();
            robot.closeUpperGlyphArms();
        }

        if (gamepad2.b) {
            robot.openLowerGlyphArms();
            robot.openUpperGlyphArms();
        }

        telemetry.update();
    }
}
