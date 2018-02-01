package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.robots.Prometheus;
import org.firstinspires.ftc.teamcode.match.Alliance;
import org.firstinspires.ftc.teamcode.match.AllianceProperties;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutoOp;
import org.firstinspires.ftc.teamcode.subsystem.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PIDComplexity;
import org.firstinspires.ftc.teamcode.subsystem.jewel.JewelSensorSystem;
import org.firstinspires.ftc.teamcode.utils.PioTimer;

@Autonomous(name = "Sensor Test - Knock Jewel Blue", group = "main")
public class JewelKnockBlue extends BaseAutoOp {

    private Alliance alliance = Alliance.BLUE;
    private Prometheus prometheus;

    @Override
    public void initializeClass() {
        prometheus = new Prometheus(this);

        setRobot(prometheus);
        setGamepadConfig(null);
        setAlliance(alliance);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        while (opModeIsActive()) {
            //Init Jewel Servo
            ((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).initializeServo();

            //Drop Jewel Arm
            ((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).dropServo();

            sleep(1500);
            if (((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).getJewel()
                    .equals(getAllianceProperties().getAllianceProperty(AllianceProperties.AllianceProperty.OPPOSITE_JEWEL_COLOR))) {

                telemetry.addData("Jewel", "Correct Color: Red");
                telemetry.update();

                //Drive 12 in. West to knock left ball
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.NORTH,
                                24 / 2,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 5, 1),
                                PIDComplexity.LOW);

                sleep(2000);
            } else if (((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).getJewel()
                    .equals(getAllianceProperties().getAllianceProperty(AllianceProperties.AllianceProperty.JEWEL_COLOR))) {

                telemetry.addData("Jewel", "Opposite Color: Red");
                telemetry.update();

                //Drive 12 in. East to knock right ball
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.SOUTH,
                                24 / 2,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 5, 1),
                                PIDComplexity.LOW);

                sleep(2000);
            }

            stop();
        }
    }
}
