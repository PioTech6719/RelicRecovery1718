package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.robots.Prometheus;
import org.firstinspires.ftc.teamcode.match.Alliance;
import org.firstinspires.ftc.teamcode.match.AllianceProperties;
import org.firstinspires.ftc.teamcode.subsystem.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PIDComplexity;
import org.firstinspires.ftc.teamcode.subsystem.jewel.JewelSensorSystem;
import org.firstinspires.ftc.teamcode.utils.PioTimer;

@Autonomous(name = "Example Auto", group = "example")
public class ExampleAutoOp extends BaseAutoOp {

    private Alliance alliance = Alliance.BLUE;
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

        waitForStart();

        while (opModeIsActive()) {
            //TODO: should include getter under prometheus instead of this?

            //Init Jewel Servo
            ((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).initializeServo();

            //Drop Jewel Arm
            ((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).dropServo();

            sleep(1500);

            //Detect Jewel Color and depending on color make specific moves
            if (((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).getJewel()
                    .equals(getAllianceProperties().getAllianceProperty(AllianceProperties.AllianceProperty.JEWEL_COLOR))) {

                //Drive 12 in. East to knock right ball
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.EAST, //TODO: Def not going to work bc encoders
                                24 / 3 * Math.sqrt(5),
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                sleep(2000);

                //Drive 24 in North to move away from board when going East
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.NORTH,
                                24 / 3,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                sleep(1500);

                //Rotate 90
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .rotateTo(-90, null, PIDComplexity.NONE);


            } else if (((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).getJewel()
                    .equals(getAllianceProperties().getAllianceProperty(AllianceProperties.AllianceProperty.OPPOSITE_JEWEL_COLOR))) {

                //Drive 12 in. West to knock left ball
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.WEST, //TODO: Def not going to work bc encoders
                                24 / 3 * Math.sqrt(5),
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                sleep(2000);

                //Drive 24 in North to move away from board when going East
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.NORTH,
                                24 / 3,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                sleep(2000);

                ((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).resetServo();

                sleep(1500);

                //Rotate 90
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .rotateTo(-90, null, PIDComplexity.NONE);

            }

            stop();
        }
    }
}
