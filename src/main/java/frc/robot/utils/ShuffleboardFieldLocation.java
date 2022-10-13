// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Cameras;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class ShuffleboardFieldLocation {

    public ShuffleboardFieldLocation() {

    }

    public static void init(Cameras cam,DriveSubsystem drive) {

        ShuffleboardLayout ft1Layout = Shuffleboard.getTab("FieldTags")
                .getLayout("1stTagRead", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

        ft1Layout.addNumber("Tag1Read X", () -> cam.tag1.getX());
        ft1Layout.addNumber("Tag1Read Y", () -> cam.tag1.getY());
        ft1Layout.addNumber("Tag1Read Z", () -> cam.tag1.getZ());

        ShuffleboardLayout ft2Layout = Shuffleboard.getTab("FieldTags")
                .getLayout("2ndTagRead", BuiltInLayouts.kList)
                .withPosition(2, 0)
                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

        ft2Layout.addNumber("Tag1Read X", () -> cam.tag2.getX());
        ft2Layout.addNumber("Tag1Read Y", () -> cam.tag2.getY());
        ft2Layout.addNumber("Tag1Read Z", () -> cam.tag2.getZ());


        ShuffleboardLayout ft3Layout = Shuffleboard.getTab("FieldTags")
                .getLayout("3rdTagRead", BuiltInLayouts.kList)
                .withPosition(4, 0)
                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

        ft3Layout.addNumber("Tag1Read X", () -> cam.tag3.getX());
        ft3Layout.addNumber("Tag1Read Y", () -> cam.tag3.getY());
        ft3Layout.addNumber("Tag1Read Z", () -> cam.tag3.getZ());


        ShuffleboardLayout robLayout = Shuffleboard.getTab("FieldTags")
                .getLayout("Robot", BuiltInLayouts.kList)
                .withPosition(6, 0)
                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

        robLayout.addNumber("Robot X", () -> drive.getX());
        robLayout.addNumber("Robot Y", () -> drive.getY());
        robLayout.addNumber("Robot Angle", () -> drive.getHeadingDegrees());

    }
}
