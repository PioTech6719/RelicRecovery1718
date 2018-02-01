package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.robots.Prometheus;
import org.firstinspires.ftc.teamcode.match.Alliance;
import org.firstinspires.ftc.teamcode.match.AllianceProperties;
import org.firstinspires.ftc.teamcode.subsystem.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PIDComplexity;
import org.firstinspires.ftc.teamcode.subsystem.glyph.GlyphSystem;
import org.firstinspires.ftc.teamcode.subsystem.jewel.JewelSensorSystem;
import org.firstinspires.ftc.teamcode.utils.PioTimer;

@Autonomous(name = "Example Auto", group = "example")
public class ExampleAutoOp extends BaseAutoOp {

    private Alliance alliance = Alliance.RED;
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
            //TODO: should include getter under prometheus instead of this?
            ((GlyphSystem) prometheus.getSubsystem(GlyphSystem.class)).initServos();

            ((GlyphSystem) prometheus.getSubsystem(GlyphSystem.class)).closeLowers();

            sleep(1000);

            //Init Jewel Servo
            ((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).initializeServo();

            //Drop Jewel Arm
            ((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).dropServo();

            sleep(1500);

            //Detect Jewel Color and depending on color make specific moves
            if (((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).getJewel()
                    .equals(getAllianceProperties().getAllianceProperty(AllianceProperties.AllianceProperty.OPPOSITE_JEWEL_COLOR))) {

                //Drive 24 in. East to knock right ball
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.NORTH, //TODO: Def not going to work bc encoders
                                24 / 2,
                                0.5,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 5, 1),
                                PIDComplexity.LOW);

                sleep(2000);

                ((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).resetServo();
                sleep(1000);

                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .rotate(null,
                                -1, //should be 0 TODO: Create mech to use the
                                null,
                                PIDComplexity.NONE);

                sleep(2000);


                //Drive 24 in North to move away from board when going East
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.WEST,
                                24 / 2 * Math.sqrt(2),
                                0.5,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 5, 1),
                                PIDComplexity.LOW);

                sleep(1500);

            } else if (((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).getJewel()
                    .equals(getAllianceProperties().getAllianceProperty(AllianceProperties.AllianceProperty.JEWEL_COLOR))) {

                //Drive 24 in. South to knock ball
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.SOUTH, //TODO: Def not going to work bc encoders
                                24 / 2,
                                0.5,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 5, 1),
                                PIDComplexity.LOW);

                sleep(2000);

                ((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).resetServo();

                sleep(1000);

                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .rotate(null,
                                0,
                                null,
                                PIDComplexity.NONE);

                sleep(2000);

                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.WEST,
                                24 / 2 * Math.sqrt(5),
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 5, 1),
                                PIDComplexity.LOW);

                sleep(2000);
            }

            //Open Lower Glyph
            ((GlyphSystem) prometheus.getSubsystem(GlyphSystem.class)).openLowers();

            ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                    .move(Directions.NORTH, //TODO: Def not going to work bc encoders
                            8 / 2,
                            0.5,
                            new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                            PIDComplexity.LOW);

            sleep(2000);

            ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                    .move(Directions.SOUTH, //TODO: Def not going to work bc encoders
                            4 / 2,
                            0.5,
                            new PioTimer(ElapsedTime.Resolution.SECONDS, 2, 1),
                            PIDComplexity.LOW);

            sleep(2000);

            stop();
        }
    }
}
