// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionPoseEstimatorSubsystem;

/** Add your docs here. */
public class ShuffleboardVisionTest {

        public static void init(VisionPoseEstimatorSubsystem vpe, DriveSubsystem drive) {
                // NetworkTableInstance inst = NetworkTableInstance.getDefault();
                // NetworkTable table = inst.getTable("PoseEst");
                // NetworkTableEntry xEntry = table.getEntry("x");
                // NetworkTableEntry yEntry = table.getEntry("y");

                ShuffleboardTab camLayout = Shuffleboard.getTab("CamerasTest");

                
                int r = 0;

                camLayout.addNumber("Target X", () -> vpe.targetPoses.get(4).getX())
                                .withPosition(0, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("Target Y", () -> vpe.targetPoses.get(4).getY())
                                .withPosition(1, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("Target Z", () -> vpe.targetPoses.get(4).getZ())
                                .withPosition(2, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("Target Angle",
                                () -> vpe.targetPoses.get(4).getRotation().getAngle())
                                .withPosition(3, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);
                // cam to target
                r = 1;
                camLayout.addNumber("CamToTarget X", () -> vpe.camToTarget[0].getX())
                                .withPosition(0, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("CamToTarget Y", () -> vpe.camToTarget[0].getY())
                                .withPosition(1, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("CamToTarget Z", () -> vpe.camToTarget[0].getZ())
                                .withPosition(2, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("CamToTarget Angle",
                                () -> vpe.camToTarget[0].getRotation().getAngle())
                                .withPosition(3, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                // cam to target
                r = 2;
                camLayout.addNumber("InvCamToTgtX", () -> vpe.camToTarget[0].inverse().getX())
                                .withPosition(0, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("InvCamToTgtY", () -> vpe.camToTarget[0].inverse().getY())
                                .withPosition(1, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("InvCamToTgtZ", () -> vpe.camToTarget[0].inverse().getZ())
                                .withPosition(2, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("InvCamToTgtAngle",
                                () -> vpe.camToTarget[0].inverse().getRotation().getAngle())
                                .withPosition(3, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                // cam pose
                r = 3;
                camLayout.addNumber("CamPose X", () -> vpe.camPose[0].getX())
                                .withPosition(0, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("CamPose Y", () -> vpe.camPose[0].getY())
                                .withPosition(1, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("CamPose Z", () -> vpe.camPose[0].getZ())
                                .withPosition(2, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("CamPose Angle",
                                () -> vpe.camPose[0].getRotation().getAngle())
                                .withPosition(3, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                // cam to target
                r = 0;
                camLayout.addNumber("VisCorr X", () -> vpe.visionMeasurement[0].getX())
                                .withPosition(4, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("VisCorr Y", () -> vpe.visionMeasurement[0].getY())
                                .withPosition(5, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("VisCorr Z", () -> vpe.visionMeasurement[0].getZ())
                                .withPosition(6, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("VisCorr Angle",
                                () -> vpe.visionMeasurement[0].getRotation().getAngle())
                                .withPosition(7, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                // robot

                r = 1;
                camLayout.addNumber("Robot X", () -> drive.getX())
                                .withPosition(4, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("Robot Y Y", () -> drive.getY())
                                .withPosition(5, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("Robot Angle",
                                () -> drive.getHeadingDegrees())
                                .withPosition(7, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);


                                // robot

                r = 2;
                camLayout.addNumber("Est X", () -> drive.m_odometry.getEstimatedPosition().getX())
                                .withPosition(4, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("Est Y ", () -> drive.m_odometry.getEstimatedPosition().getY())
                                .withPosition(5, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                camLayout.addNumber("Est Angle",
                                () -> drive.m_odometry.getEstimatedPosition().getRotation().getDegrees())
                                .withPosition(7, r).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

        }

}