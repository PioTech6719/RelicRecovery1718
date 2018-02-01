package org.firstinspires.ftc.teamcode.opmodes.auto.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.robots.Prometheus;
import org.firstinspires.ftc.teamcode.match.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutoOp;
import org.firstinspires.ftc.teamcode.subsystem.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PIDComplexity;
import org.firstinspires.ftc.teamcode.subsystem.sensors.gyro.GyroSensorSystem;

@Autonomous(name = "Sesnor Test - Simple Rotation - PASSED", group = "test")
public class RotationTest extends BaseAutoOp {

    private Alliance alliance = Alliance.RED;
    private Prometheus prometheus;

    @Override
    public void initializeClass() {
        prometheus = new Prometheus(this);

        setRobot(prometheus);
        setGamepadConfig(null);
        setAlliance(Alliance.BLUE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        telemetry.addData("Status : ", "Do NOT have Timer on");
        telemetry.update();

        GyroSensorSystem gyroSensorSystem = (GyroSensorSystem) prometheus.getSubsystem(GyroSensorSystem.class);

        waitForStart();

        while (opModeIsActive()) {
            ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                    .rotate(null,
                            -90,
                            null,
                            PIDComplexity.NONE);

            sleep(2000);

            ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                    .rotate(null,
                            90,
                            null,
                            PIDComplexity.NONE);

            sleep(2000);

            stop();
        }
    }
}