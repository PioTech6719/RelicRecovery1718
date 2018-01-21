package org.firstinspires.ftc.teamcode.opmodes.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotTelemetry {

    public static String NEW_LINE_SEPERATOR = "/n";
    private static Telemetry telemetry;
    private static String TAG = "";
    private static String currentText = "";

    public RobotTelemetry(OpMode opMode, String TAG) {
        RobotTelemetry.TAG = TAG;
        RobotTelemetry.telemetry = opMode.telemetry;

        initializeTelemetrySettings();
    }

    public static void append(String... text) {
        clear();
        format(TAG, currentText);

        for (String line : text) {
            format(TAG, line);
        }

        update();
    }

    public static void append(String customTag, String... text) {
        clear();
        format(customTag, currentText);

        for (String line : text) {
            format(customTag, line);
        }

        update();
    }

    public static void write(String... text) {
        clear();

        for (String line : text) {
            format(TAG, line);
        }

        update();
    }

    public static void write(String customTag, String... text) {
        clear();

        for (String line : text) {
            format(customTag, line);
        }

        update();
    }

    public static void clear() {
        telemetry.clearAll();
        currentText = "";
    }

    public static void update() {
        telemetry.update();
    }

    /**
     * Checks for special formatting symbols
     */
    private static void format(String tag, String text) {
        String newText = text;
        int lineSeperatorCount = 0;
        while (newText.contains(NEW_LINE_SEPERATOR)) {
            lineSeperatorCount++;
            newText = newText.substring(newText.indexOf(NEW_LINE_SEPERATOR) + NEW_LINE_SEPERATOR.length());
        }

        if (lineSeperatorCount == 0) {
            telemetry.addData(tag, text);
        }

        String remainderText = text;
        while (lineSeperatorCount > 0) {
            telemetry.addData(tag, remainderText.substring(0, remainderText.indexOf(NEW_LINE_SEPERATOR)));
            telemetry.addLine();
            remainderText = remainderText.substring(remainderText.indexOf(NEW_LINE_SEPERATOR) + NEW_LINE_SEPERATOR.length());
            lineSeperatorCount--;
        }

        currentText += text;
    }

    private void initializeTelemetrySettings() {
        telemetry.setAutoClear(false);
    }
}
