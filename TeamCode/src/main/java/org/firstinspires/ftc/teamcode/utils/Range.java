package org.firstinspires.ftc.teamcode.utils;

public class Range {

    public static boolean inRange(double val, double min, double max, boolean inclusive) {
        if (inclusive) {
            return val >= min && val <= max;
        } else {
            return val > min && val < max;
        }
    }
}
