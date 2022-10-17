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

        public static void init(Cameras cam, DriveSubsystem drive) {

                ShuffleboardLayout ft1Layout = Shuffleboard.getTab("FieldTags")
                                .getLayout("1stTagData", BuiltInLayouts.kList)
                                .withPosition(0, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                ft1Layout.addNumber("Tag1Read X", () -> AprilTagData.get3dData(cam.tagID[0])[1]);
                ft1Layout.addNumber("Tag1Read Y", () -> AprilTagData.get3dData(cam.tagID[0])[2]);
                ft1Layout.addNumber("Tag1Read Z", () -> AprilTagData.get3dData(cam.tagID[0])[3]);
                ft1Layout.addNumber("Tag1Read ZRot", () -> AprilTagData.get3dData(cam.tagID[0])[4]);
                ft1Layout.addNumber("Tag1Read YRot", () -> AprilTagData.get3dData(cam.tagID[0])[5]);
                ft1Layout.addString("Tag1Location", () -> AprilTagData.getTagLocation(cam.tagID[0]));

                ShuffleboardLayout ft2Layout = Shuffleboard.getTab("FieldTags")
                                .getLayout("2ndTagData", BuiltInLayouts.kList)
                                .withPosition(2, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                ft2Layout.addNumber("Tag2Read X", () -> AprilTagData.get3dData(cam.tagID[1])[1]);
                ft2Layout.addNumber("Tag2Read Y", () -> AprilTagData.get3dData(cam.tagID[1])[2]);
                ft2Layout.addNumber("Tag2Read Z", () -> AprilTagData.get3dData(cam.tagID[1])[3]);
                ft2Layout.addNumber("Tag2Read ZRot", () -> AprilTagData.get3dData(cam.tagID[1])[4]);
                ft2Layout.addNumber("Tag2Read YRot", () -> AprilTagData.get3dData(cam.tagID[1])[5]);
                ft2Layout.addString("Tag2Location", () -> AprilTagData.getTagLocation(cam.tagID[1]));

                ShuffleboardLayout ft3Layout = Shuffleboard.getTab("FieldTags")
                                .getLayout("3rdTagData", BuiltInLayouts.kList)
                                .withPosition(4, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                ft3Layout.addNumber("Tag3Read X", () -> AprilTagData.get3dData(cam.tagID[2])[1]);
                ft3Layout.addNumber("Tag3Read Y", () -> AprilTagData.get3dData(cam.tagID[2])[2]);
                ft3Layout.addNumber("Tag3Read Z", () -> AprilTagData.get3dData(cam.tagID[2])[3]);
                ft3Layout.addNumber("Tag3Read ZRot", () -> AprilTagData.get3dData(cam.tagID[2])[4]);
                ft3Layout.addNumber("Tag3Read YRot", () -> AprilTagData.get3dData(cam.tagID[2])[5]);
                ft3Layout.addString("Tag3Location", () -> AprilTagData.getTagLocation(cam.tagID[2]));

                ShuffleboardLayout robLayout = Shuffleboard.getTab("FieldTags")
                                .getLayout("Robot", BuiltInLayouts.kList)
                                .withPosition(6, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                robLayout.addNumber("Robot X", () -> drive.getX());
                robLayout.addNumber("Robot Y", () -> drive.getY());
                robLayout.addNumber("Robot Angle", () -> drive.getHeadingDegrees());

                ShuffleboardLayout t0L3d;
                ShuffleboardLayout t1L3d;
                ShuffleboardLayout t2L3d;

                t0L3d = Shuffleboard.getTab("FieldTags")
                                .getLayout("BestTarget3D", BuiltInLayouts.kList).withPosition(0, 2)

                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                // if (cam.use3D)
                write3DValues(t0L3d, cam, 0);

                t1L3d = Shuffleboard.getTab("FieldTags")
                                .getLayout("Second Target3D", BuiltInLayouts.kList).withPosition(2, 2)

                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));
                // if (cam.use3D)
                write3DValues(t1L3d, cam, 1);

                t2L3d = Shuffleboard.getTab("FieldTags")
                                .getLayout("ThirdTarget3D", BuiltInLayouts.kList).withPosition(4, 2)

                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));
                // if (cam.use3D)
                write3DValues(t2L3d, cam, 2);
        }

        public static void write3DValues(ShuffleboardLayout tnL, Cameras cam, int n) {

                tnL.addNumber("3D-ToCam- X", () -> cam.X[n]);

                tnL.addNumber("3D-ToCam- Y", () -> cam.Y[n]);

                tnL.addNumber("3D-ToCam- Z", () -> cam.Z[n]);

                tnL.addNumber("3D-ToCam Angle", () -> cam.A[n]);

               
        }
}
