package org.firstinspires.ftc.teamcode.subsystem.sensors.camera;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;

public class VuforiaSystem extends Subsystem {

    private ClosableVuforiaLocalizer vuforia;
    private String VUFORIA_KEY = "Ac2N2pD/////AAAAGUtP1PV5KUFbn2hG/K+RFO1V1MLAkEORx2m1ZGTlPO/Myj" +
            "NlkL/iv3eFSj9um8RlJ0HrOsHSJ3ajrpZgyAnyikkmh2rPyWbmw5UfLw9YLLGsC99ATkXLKk+sjjiUTSV0" +
            "hubnL+9AfgrvhNY4kTG3XX7feZvFByyF1La4Inw/UFPx9LiunShdUK9QRgX3HfSACm1JHZGVj8mL0TByq6" +
            "OL4lOA77UOX8Kn++Ed/OMyv6QyaRVxGob8uQv+xBhvKKm1QUFg2tTRHdnV9Lc0tbMUQLx2bS6XgAIkq4qF" +
            "8ZIMOcd8uUYztkKoI5V9kiOHFpNZJS8yE7Qi8pgdwuetwHO3X4ZKLMUwBnUYVQbqfPzOa2ay";
    private VuforiaTrackable relicTemplate;

    private boolean visibleView;

    public VuforiaSystem(Robot robot, boolean visibleView) {
        super(robot);
        this.visibleView = visibleView;

        //Intentionally didn't call init() as may clash with other CV platform
    }

    @Override
    public void init() {
        VuforiaLocalizer.Parameters parameters;

        if (visibleView) {
            //Place view onto RC
            int cameraMonitorViewId = Robot.getOpMode().hardwareMap.appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id", Robot.getOpMode().hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        this.vuforia = new ClosableVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {
    }

    @Override
    public void stop() {
        closeVuforia();
    }

    public RelicRecoveryVuMark getVuMark() {
        return RelicRecoveryVuMark.from(relicTemplate);
    }

    private void closeVuforia() {
        vuforia.close();
    }
}
