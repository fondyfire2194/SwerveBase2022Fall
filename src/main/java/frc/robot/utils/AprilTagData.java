// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class AprilTagData {
    public static int n;

    public static Translation3d tagData[] = new Translation3d[4];

    public AprilTagData() {
    }

    public static void init() {

        tagData[0] = new Translation3d(2, 0, 1);
        tagData[1] = new Translation3d(3, 0, 1);
        tagData[2] = new Translation3d(0, 1, 1);
        tagData[3] = new Translation3d(0, 1, 1);

    }

    public static Translation3d getTranslation3d(int n) {

        if (n < 0)
            n = 0;

        if (n > tagData.length - 1)
            n = 0;

        return tagData[n];
    }

}