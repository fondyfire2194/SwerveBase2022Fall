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
import frc.robot.subsystems.PoseEstimatorSubsystem;

/** Add your docs here. */
public class ShuffleboardFieldLocation {

        public ShuffleboardFieldLocation() {

        }

        public static void init(Cameras cam, DriveSubsystem drive, PoseEstimatorSubsystem pos) {

                ShuffleboardLayout robLayout = Shuffleboard.getTab("FieldTags")
                                .getLayout("RobotToField", BuiltInLayouts.kList)
                                .withPosition(8, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                robLayout.addNumber("Robot X", () -> drive.getX());
                robLayout.addNumber("Robot Y", () -> drive.getY());
                robLayout.addNumber("Robot Angle", () -> drive.getHeadingDegrees());

                ShuffleboardLayout robToCamLayout = Shuffleboard.getTab("FieldTags")
                                .getLayout("RobotToCam", BuiltInLayouts.kList)
                                .withPosition(8, 2)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                robToCamLayout.addNumber("CamToRob X", () -> pos.CAMERA_TO_ROBOT.getX());
                robToCamLayout.addNumber("CamToRob Y", () -> pos.CAMERA_TO_ROBOT.getY());
                robToCamLayout.addNumber("CamToRob Angle", () -> pos.CAMERA_TO_ROBOT.getRotation().getDegrees());

                ShuffleboardLayout t0L3d = Shuffleboard.getTab("FieldTags")
                                .getLayout("Target 1", BuiltInLayouts.kList).withPosition(0, 0)

                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                t0L3d.addNumber("FieldX", () -> pos.targetPoses.get(0).getTranslation().getX());
                t0L3d.addNumber("FieldY", () -> pos.targetPoses.get(0).getTranslation().getY());
                t0L3d.addNumber("FieldAngle", () -> pos.targetPoses.get(0).getRotation().getDegrees());

                ShuffleboardLayout t1L3d = Shuffleboard.getTab("FieldTags")
                                .getLayout("Target 2", BuiltInLayouts.kList).withPosition(0, 2)

                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                t1L3d.addNumber("FieldX", () -> pos.targetPoses.get(1).getTranslation().getX());
                t1L3d.addNumber("FieldY", () -> pos.targetPoses.get(1).getTranslation().getY());
                t1L3d.addNumber("FieldAngle", () -> pos.targetPoses.get(1).getRotation().getDegrees());

                ShuffleboardLayout ft1Layout = Shuffleboard.getTab("FieldTags")
                                .getLayout("CamToTarget 1", BuiltInLayouts.kList)
                                .withPosition(4, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                ft1Layout.addNumber("FiducialD1", () -> pos.target[0].getFiducialId());

                ft1Layout.addNumber("CtoT1-X", () -> pos.camToTarget[0].getX());

                ft1Layout.addNumber("CtoT1-Y", () -> pos.camToTarget[0].getY());

                ft1Layout.addNumber("CtoT1-Z", () -> pos.camToTarget[0].getZ());

                ft1Layout.addNumber("CToTIAngle", () -> pos.camToTarget[0].getRotation().getAngle());

                ShuffleboardLayout ft2Layout = Shuffleboard.getTab("FieldTags")
                                .getLayout("CamToTarget 2", BuiltInLayouts.kList)
                                .withPosition(4, 2)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                ft2Layout.addNumber("FiducialD2", () -> pos.target[1].getFiducialId());

                ft2Layout.addNumber("CtoT2-X", () -> pos.camToTarget[1].getX());

                ft2Layout.addNumber("CtoT2-Y", () -> pos.camToTarget[1].getY());

                ft2Layout.addNumber("CtoT2-Z", () -> pos.camToTarget[1].getZ());

                ft2Layout.addNumber("CtoT2Angle", () -> pos.camToTarget[1].getRotation().getAngle());
                

                ShuffleboardLayout ft11Layout = Shuffleboard.getTab("FieldTags")
                                .getLayout("RobToTarget 1", BuiltInLayouts.kList)
                                .withPosition(2, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                ft11Layout.addNumber("RobtoT1X", () -> pos.target[0].getCameraToTarget().getX());

                ft11Layout.addNumber("RobtoT1Y", () -> pos.target[0].getCameraToTarget().getY());

                ft11Layout.addNumber("RobtoT1Z", () -> pos.target[0].getCameraToTarget().getZ());

                ft11Layout.addNumber("RobtoT1Agle", () -> pos.target[0].getCameraToTarget().getRotation().getAngle());

                ShuffleboardLayout ft21Layout = Shuffleboard.getTab("FieldTags")
                                .getLayout("RobToTarget 2", BuiltInLayouts.kList)
                                .withPosition(2, 2)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                ft21Layout.addNumber("RobtoT2X", () -> pos.target[1].getCameraToTarget().getX());

                ft21Layout.addNumber("RobtoT2Y", () -> pos.target[1].getCameraToTarget().getY());

                ft21Layout.addNumber("RobtoT2Z", () -> pos.target[1].getCameraToTarget().getZ());

                ft21Layout.addNumber("RobtoT2Angle", () -> pos.target[1].getCameraToTarget().getRotation().getAngle());

        }

}
