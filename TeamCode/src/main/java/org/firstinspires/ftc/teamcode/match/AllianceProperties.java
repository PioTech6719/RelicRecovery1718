package org.firstinspires.ftc.teamcode.match;

import org.firstinspires.ftc.teamcode.subsystem.jewel.Jewel;

public class AllianceProperties {

    private Alliance alliance;

    public AllianceProperties(Alliance alliance) {
        this.alliance = alliance;
    }

    public Object getAllianceProperty(AllianceProperty allianceProperty) {
        switch (allianceProperty) {

            case JEWEL_COLOR:
                return getDesiredJewel();
            case OPPOSITE_JEWEL_COLOR:
                return getOppositeJewel();
        }

        return null;
    }

    private Jewel getDesiredJewel() {
        if (alliance.equals(Alliance.BLUE)) {
            return Jewel.BLUE;
        } else if (alliance.equals(Alliance.RED)) {
            return Jewel.RED;
        }

        return null;
    }

    private Jewel getOppositeJewel() {
        if (alliance.equals(Alliance.BLUE)) {
            return Jewel.RED;
        } else if (alliance.equals(Alliance.RED)) {
            return Jewel.BLUE;
        }

        return null;
    }

    public enum AllianceProperty {
        JEWEL_COLOR, OPPOSITE_JEWEL_COLOR
    }
}
