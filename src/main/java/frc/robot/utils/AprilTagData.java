// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class AprilTagData {

    public static int n;
    public static int highestTagNumber = 46;
    private static int lastLowTag = 17;
    private static int firstHighTag = 40;
    private static int highLowTagGap = firstHighTag - lastLowTag;
    // from Rapid React post season
    public static double[][] tagLocationData = {

        //tagid#,Xm,Ym,Zm,Zrot,YRot,XRot

            { 0, 0, 0, 0, 0, 0, 0 }, // not used
            { 1, -0.004, 7.58, 0.89, 0, 0.00, 0.00 }, // tag 1
            { 2, 3.233, 5.49, 1.73, 0, 0.00, 0.00 },
            { 3, 3.068, 5.33, 1.38, 0, 90.00, 0.00 },
            { 4, 0.004, 5.06, 0.81, 0, 0.00, 0.00 },
            { 5, 0.004, 3.51, 0.81, 0, 0.00, 0.00 },
            { 6, 0.121, 1.72, 0.89, 0, 46.25, 0.00 },
            { 7, 0.873, 0.94, 0.89, 0, 46.25, 0.00 },
            { 8, 1.615, 0.16, 0.89, 0, 46.25, 0.00 },
            { 9, 16.463, 0.65, 0.89, 0, 180.00, 0.00 },
            { 10, 13.235, 2.74, 1.73, 0, 180.00, 0.00 },
            { 11, 13.391, 2.90, 1.38, 0, 180.00, 0.00 },
            { 12, 16.455, 3.18, 0.81, 0, 180.00, 0.00 },
            { 13, 16.455, 4.72, 0.81, 0, 180.00, 0.00 },
            { 14, 16.335, 6.51, 0.89, 0, 223.80, 0.00 },
            { 15, 15.591, 7.29, 0.89, 0, 223.80, 0.00 },
            { 16, 14.847, 8.07, 0.89, 0, 223.8, 0.00 },
            { 17, 7.874, 4.91, 0.70, 0, 114.00, 0.00 }, // 16

            { 40, 7.431, 3.76, 0.70, 0, 204.00, 0.00 }, // 17 = tag 40
            { 41, 8.585, 3.32, 0.70, 0, -66.00, 0.00 },
            { 42, 9.028, 4.47, 0.70, 0, 24.00, 0.00 },
            { 43, 7.679, 4.33, 2.42, 0, 159.00, 26.75 },
            { 44, 8.018, 3.56, 2.42, 0, 339.00, 26.75 },
            { 45, 8.780, 3.90, 2.42, 0, 249.00, 26.75 },
            { 46, 8.441, 4.67, 2.42, 0, 69.00, 26.75 }
    };

    public static String[] tagLocations = {
            "",
            "Blue Hangar Truss - Hub",
            "Blue Hangar Truss - Side",
            "Blue Station 2 Wall",
            "Blue Station 3 Wall",
            "Blue Terminal Near Station",
            "Blue Mid Terminal",
            "Blue End Terminal",
            "Red Hangar Panel",
            "Red Hangar Truss - Hub",
            "Red Hangar Truss - Side",
            "Red Station 2 Wall",
            "Red Station 3 Wall",
            "Red Terminal Near Station",
            "Red Mid Terminal",
            "Red End Terminal",
            "Lower Hub Far Exit",
            "Lower Hub Blue Exit",
            "Lower Hub Near Exit",
            "Lower Hub Red Exit",
            "Upper Hub Far-Blue",
            "Upper Hub Blue-Near",
            "Upper Hub Near-Red",
            "Upper Hub Red-Far"

    };

    public AprilTagData() {

    }

    public static Transform3d getTransform3d(int n) {

        if (!getValidTargetNumber(n))

            return new Transform3d();

        else {

            if (n >= firstHighTag)

                n -= highLowTagGap;
        }

        double x = tagLocationData[n][1];
        double y = tagLocationData[n][2];
        double z = tagLocationData[n][3];
         double roll = tagLocationData[n][4];
        double Z_rot = tagLocationData[n][5];
        double Y_rot = tagLocationData[n][6];

        return new Transform3d(new Translation3d(x, y, z), new Rotation3d(0, Z_rot, Y_rot));
    }

    public static double[] get3dData(int n) {

        double temp[] = { 0, 0, 0, 0, 0, 0, 0 };

        if (!getValidTargetNumber(n))

            return temp;

        else {

            if (n >= firstHighTag)

                n -= highLowTagGap;
        }

        temp[0] = tagLocationData[n][0];
        temp[1] = tagLocationData[n][1];
        temp[2] = tagLocationData[n][2];
        temp[3] = tagLocationData[n][3];
        temp[4] = tagLocationData[n][4];
        temp[5] = tagLocationData[n][5];
        temp[6] = tagLocationData[n][6];

        return temp;
    }

    public static String getTagLocation(int n) {

        if (!getValidTargetNumber(n))

            return "Not a Location";

        else {

            if (n >= firstHighTag)

                n -= highLowTagGap;
        }

        return tagLocations[n];

    }

    public static boolean getValidTargetNumber(int i) {
        boolean lowTargetGood = i > 0 && i <= lastLowTag;
        SmartDashboard.putBoolean("LO", lowTargetGood);
        boolean highTargetGood = i >= firstHighTag && i <= highestTagNumber;
        SmartDashboard.putBoolean("HI", highTargetGood);
        return lowTargetGood || highTargetGood;
    }

}