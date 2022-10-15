// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class AprilTagData {

    public static int n;

    public static double[][] tagLocations = {

            { 1, 2, 0, 0, 0, 0 }, // tag 0
            { 1, 2, 45, 0, 0, 1 },
            { 4, 5, 30, 0, 0, 2 }, // x,y,z,roll,pitch,yae //meters/ degrees
            { 7, 8, 0, 0, 0, 3 },
            { 7, 8, 0, 0, 0, 4 },
            { 1, 2, 90, 0, 0, 5 },
            { 1, 2, 90, 0, 0, 6 },
            { 1, 2, 0, 0, 0, 7 },
            { 1, 2, 45, 0, 0, 8 },
            { 1, 2, 30, 9, 0, 0 }// tag 9

    };

    public AprilTagData() {

    }

    public static Transform3d getTransform3d(int n) {

        if (n < 0 || n > tagLocations.length - 1)
            return new Transform3d();
        else {
            double x = tagLocations[n][0];
            double y = tagLocations[n][1];
            double z = tagLocations[n][2];
            double roll = tagLocations[n][3];
            double pitch = tagLocations[n][4];
            double yaw = tagLocations[n][5];

            return new Transform3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
        }
    }

}