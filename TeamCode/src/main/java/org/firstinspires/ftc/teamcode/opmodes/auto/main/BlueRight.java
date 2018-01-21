package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.hardware.robots.Prometheus;
import org.firstinspires.ftc.teamcode.match.Alliance;
import org.firstinspires.ftc.teamcode.match.AllianceProperties;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutoOp;
import org.firstinspires.ftc.teamcode.subsystem.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PIDComplexity;
import org.firstinspires.ftc.teamcode.subsystem.glyph.GlyphSystem;
import org.firstinspires.ftc.teamcode.subsystem.jewel.JewelSensorSystem;
import org.firstinspires.ftc.teamcode.utils.PioTimer;

@Autonomous(name = "Blue Right Auto", group = "main")
public class BlueRight extends BaseAutoOp {

    private final String TELEMETRY_TAG = "Status: ";
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
        telemetry.addData(TELEMETRY_TAG, "Initlializing... Let's GET ITTTTTTTTTTTTTTTTTTTTTTTTTTT");
        telemetry.update();

        super.runOpMode();

        telemetry.addData(TELEMETRY_TAG, "Initialized ... boutta kill");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData(TELEMETRY_TAG, "Starting peeps");
            telemetry.update();

            //Init Glyph Servos
            ((GlyphSystem) prometheus.getSubsystem(GlyphSystem.class)).initServos();

            telemetry.addData(TELEMETRY_TAG, "Initialized Glyph Servos");
            telemetry.update();

            //Lock onto Glyph
            ((GlyphSystem) prometheus.getSubsystem(GlyphSystem.class)).closeLowers();

            telemetry.addData(TELEMETRY_TAG, "Closed Lower Glyph Servos");
            telemetry.update();

            sleep(1000);

            //Lift Glyph up 4 inches
//            ((LiftSystem) prometheus.getSubsystem(LiftSystem.class)).moveLift(1, 4); //TODO: Check and verify the power and direction this
//
//            telemetry.addData(TELEMETRY_TAG, "Raising Lift");
//            telemetry.update();
//
//            sleep(1500);

            //Init Jewel Servo
            ((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).initializeServo();

            telemetry.addData(TELEMETRY_TAG, "Initialized Jewel Servo");
            telemetry.update();

            //Drop Jewel Arm and wait to detect the Color
            ((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).dropServo();

            telemetry.addData(TELEMETRY_TAG, "Dropped Servo and Scanning Color");
            telemetry.update();

            sleep(1500);

            telemetry.addData(TELEMETRY_TAG, "Read Right Ball Color : ");
            telemetry.addData(TELEMETRY_TAG, new Func<String>() {
                @Override
                public String value() {
                    switch (((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).getJewel()) {
                        case RED:
                            return "RED";
                        case BLUE:
                            return "BLUE";
                        case NONE:
                            return "NONE";
                    }
                    return "NONE";
                }
            });
            telemetry.update();

            //Detect Jewel Color and depending on color make specific moves
            if (((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).getJewel()
                    .equals(getAllianceProperties().getAllianceProperty(AllianceProperties.AllianceProperty.OPPOSITE_JEWEL_COLOR))) {

                telemetry.addData(TELEMETRY_TAG, "Right Ball is Correct Color");
                telemetry.update();

                //Drive East to knock right ball
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.EAST,
                                12 / 3 * Math.sqrt(5),
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                telemetry.addData(TELEMETRY_TAG, "Driving East to knock right ball");
                telemetry.update();

                sleep(2000);

                //Drive North away from board
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.NORTH,
                                24 / 3,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                telemetry.addData(TELEMETRY_TAG, "Driving North away from board");
                telemetry.update();

                sleep(2000);

                //Drive West to move to crypto
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.WEST,
                                48 / 3 * Math.sqrt(5),
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                telemetry.addData(TELEMETRY_TAG, "Driving West to crypto");
                telemetry.update();

                sleep(2000);

                //TODO: Align straight
                PrometheusConstants.TOLERANCE = 4;
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .rotateTo(0, null, PIDComplexity.NONE);

                telemetry.addData(TELEMETRY_TAG, "Facing Glyphs");
                telemetry.update();

                sleep(1000);

                //TODO: Lower tolerance
                PrometheusConstants.TOLERANCE = 4;
                //Rotate 90 to face Crypto and align
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .rotateTo(180, null, PIDComplexity.NONE);

                telemetry.addData(TELEMETRY_TAG, "Facing Crypto and align");
                telemetry.update();

                //Drive 12 in North to move next to Cryptobox
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.NORTH,
                                24 / 3,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                telemetry.addData(TELEMETRY_TAG, "Driving North to move next to Cryptobox");
                telemetry.update();

                sleep(1500);

                //TODO: Use VuMark to determine dist and direction east or west

//                ((LiftSystem) prometheus.getSubsystem(LiftSystem.class)).moveLift(-1, 4);
//
//                telemetry.addData(TELEMETRY_TAG, "Lowering Lift");
//                telemetry.update();
//
//                sleep(1500);

                //Release Glyph
                ((GlyphSystem) prometheus.getSubsystem(GlyphSystem.class)).openLowers();

                telemetry.addData(TELEMETRY_TAG, "Release Lower Glyph Servos");
                telemetry.update();

                sleep(1000);

                //Drive 12 in North to push block into cryptobox
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.NORTH,
                                12 / 3,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                telemetry.addData(TELEMETRY_TAG, "Driving North to push glyph into cryptobox");
                telemetry.update();

                sleep(1500);

                //Drive 12 in South to move away from glyph
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.SOUTH,
                                12 / 3,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                telemetry.addData(TELEMETRY_TAG, "Driving South to move a bit away from glyph");
                telemetry.update();

                sleep(1500);

                //Rotate 90 to face glyphs
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .rotateTo(0, null, PIDComplexity.NONE);

                telemetry.addData(TELEMETRY_TAG, "Facing Glyphs");
                telemetry.update();

                //Opposite Jewel Color
            } else if (((JewelSensorSystem) prometheus.getSubsystem(JewelSensorSystem.class)).getJewel()
                    .equals(getAllianceProperties().getAllianceProperty(AllianceProperties.AllianceProperty.JEWEL_COLOR))) {

                telemetry.addData(TELEMETRY_TAG, "Left Ball is Correct Color");
                telemetry.update();

                //Drive West to knock right ball
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.WEST,
                                36 / 3 * Math.sqrt(5),
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 4, 1),
                                PIDComplexity.LOW);

                telemetry.addData(TELEMETRY_TAG, "Driving WEST to knock left ball");
                telemetry.update();

                sleep(2000); //TODO: -mb causing error

                //TODO: Align straight with low tolerance

                PrometheusConstants.TOLERANCE = 3;
                //Rotate 90 to face cryptobox
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .rotateTo(-180, null, PIDComplexity.NONE);

                telemetry.addData(TELEMETRY_TAG, "Facing Cryptobox");
                telemetry.update();
                PrometheusConstants.TOLERANCE = 5;
                sleep(1500);

                //TODO: Use VuMark to determine dist and direction east or west

//                ((LiftSystem) prometheus.getSubsystem(LiftSystem.class)).moveLift(-1, 4);
//
//                telemetry.addData(TELEMETRY_TAG, "Lowering Lift");
//                telemetry.update();
//
//                sleep(1500);

                //Release Glyph
                ((GlyphSystem) prometheus.getSubsystem(GlyphSystem.class)).openLowers();

                telemetry.addData(TELEMETRY_TAG, "Release Lower Glyph Servos");
                telemetry.update();

                sleep(1000);

                //Drive 12 in North to push block into cryptobox
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.NORTH,
                                18 / 3,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                telemetry.addData(TELEMETRY_TAG, "Driving North to push glyph into cryptobox");
                telemetry.update();

                sleep(1500);

                //Drive 12 in South to move away from glyph
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.SOUTH,
                                6 / 3,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                telemetry.addData(TELEMETRY_TAG, "Driving South to move a bit away from glyph");
                telemetry.update();

                sleep(1500);

                //Rotate -180 to face glyphs
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .rotateTo(0, null, PIDComplexity.NONE);

                telemetry.addData(TELEMETRY_TAG, "Facing Glyphs");
                telemetry.update();
            }

            stop();
        }
    }
}
