package org.bitbuckets;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utilities for making this work! None of the problems solutions will be found here!
 */
public class CorrectCode {


    static double[] makePoseArray(double x, double y, double rotation_rads) {
        return new double[] {x,y,rotation_rads};
    }

    static void logPoseArray(double[] poseArray) {
        SmartDashboard.getEntry("robot-pose").setDoubleArray(poseArray);
    }

}
