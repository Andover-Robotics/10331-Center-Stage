/*package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import java.util.HashMap;
import java.util.Map;
/**
 * Collection of utilites for interacting with Lynx modules.

public class LynxModuleUtil {
    private static final LynxFirmwareVersion MIN_VERSION = new LynxFirmwareVersion(1, 8, 2);
    /**
     * Parsed representation of a Lynx module firmware version.

    public static class LynxFirmwareVersion implements Comparable<LynxFirmwareVersion> {
        public final int major;
        public final int minor;
        public final int eng;
        public LynxFirmwareVersion(int major, int minor, int eng) {
            this.major = major;
            this.minor = minor;
            this.eng = eng;
        }
        @Override
        public boolean equals(Object other) {
            if (other instanceof LynxFirmwareVersion) {
                LynxFirmwareVersion otherVersion = (LynxFirmwareVersion) other;
                return major == otherVersion.major && minor == otherVersion.minor &&
                        eng == otherVersion.eng;
            } else {
                return false;
            }
        }
        @Override
        public int compareTo(LynxFirmwareVersion other) {
            int majorComp = Integer.compare(major, other.major);
            if (majorComp == 0) {
                int minorComp = Integer.compare(minor, other.minor);
                if (minorComp == 0) {
                    return Integer.compare(eng, other.eng);
                } else {
                    return minorComp;
                }


 */