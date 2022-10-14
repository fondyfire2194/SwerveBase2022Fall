// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class AprilTagData {

    public static int n;

    public static double[][] tagLocations = {

            { 1, 2, 0 }, // tag 0
            { 1, 2, 1 },
            { 4, 5, 2 }, // x,y,z ,meters
            { 7, 8, 3 },
            { 7, 8, 4 },
            { 1, 2, 5 },
            { 1, 2, 6 },
            { 1, 2, 7 },
            { 1, 2, 8 },
            { 1, 2, 9 }// tag 9

    };

    public AprilTagData() {

    }

    public static Translation3d getTranslation3d(int n) {

        if (n < 0 || n > tagLocations.length - 1)
            return new Translation3d(9, 1, 1);
        else {
            double x = tagLocations[n][0];
            double y = tagLocations[n][1];
            double z = tagLocations[n][2];
            return new Translation3d(x, y, z);
        }
    }

}