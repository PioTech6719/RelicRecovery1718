package org.firstinspires.ftc.teamcode.opmodes.tele.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.Prometheus;

import java.util.Locale;

@TeleOp(name = "Sensor - REV Gyro Sensor", group = "test")
public class REVGyroTest extends OpMode {

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    private Prometheus robot = new Prometheus();

    @Override
    public void init() {
        robot.init(hardwareMap);

        angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = robot.imu.getGravity();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Prometheus Controller app on the phone).
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) { //go forward
            robot.frontLeft.setPower(1.0);
            robot.frontRight.setPower(1.0);
            robot.backLeft.setPower(1.0);
            robot.backRight.setPower(1.0);
        } else if (gamepad1.dpad_down) {
            robot.frontLeft.setPower(-1.0);
            robot.frontRight.setPower(-1.0);
            robot.backLeft.setPower(-1.0);
            robot.backRight.setPower(-1.0);
        } else if (gamepad1.dpad_left) {
            robot.frontLeft.setPower(-1.0);
            robot.frontRight.setPower(1.0);
            robot.backLeft.setPower(1.0);
            robot.backRight.setPower(-1.0);
        } else if (gamepad1.dpad_right) {
            robot.frontLeft.setPower(1.0);
            robot.frontRight.setPower(-1.0);
            robot.backLeft.setPower(-1.0);
            robot.backRight.setPower(1.0);
        } else if (gamepad1.left_trigger > 0) { //Rotate Left
            robot.frontLeft.setPower(-0.4);
            robot.frontRight.setPower(0.4);
            robot.backLeft.setPower(-0.4);
            robot.backRight.setPower(0.4);
        } else if (gamepad1.right_trigger > 0) { //Rotate Right
            robot.frontLeft.setPower(0.4);
            robot.frontRight.setPower(-0.4);
            robot.backLeft.setPower(0.4);
            robot.backRight.setPower(-0.4);
        } else if (gamepad1.a) {
            robot.imuRotateTo(90, Directions.ROTATE_CCW, 5);
        } else if (gamepad1.b) {
            robot.imuRotateTo(-90, Directions.ROTATE_CW, 5);
        } else if (gamepad1.x) {
            robot.imuRotateTo(90, 5);
        } else if (gamepad1.y) {
            robot.imuRotateTo(150, 5);
        } else if (gamepad1.right_bumper) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        } else {
            robot.frontLeft.setPower(0.0);
            robot.frontRight.setPower(0.0);
            robot.backLeft.setPower(0.0);
            robot.backRight.setPower(0.0);
        }

        telemetry.clearAll();
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        telemetry.update();
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
