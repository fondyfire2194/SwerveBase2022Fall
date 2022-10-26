// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionPoseEstimatorSubsystem;

public class VisionSim {
  private final DriveSubsystem m_drive;

  private final VisionPoseEstimatorSubsystem m_visPosEst;

  private final Field2d m_field2d = new Field2d();

   public VisionSim(DriveSubsystem swerveDrive, VisionPoseEstimatorSubsystem visPosEst) {
    m_drive = swerveDrive;
    m_visPosEst = visPosEst;

    m_field2d.getObject("Target 1").setPose(m_visPosEst.targetPoses.get(4).toPose2d());

    m_field2d.getObject("Target 2").setPose(m_visPosEst.targetPoses.get(2).toPose2d());

    m_drive.m_odometry.resetPosition(m_visPosEst.targetPoses.get(5).toPose2d(), new Rotation2d());

  }

   public void initSim() {
  }

  public Field2d getField2d() {

    return m_field2d;
  }

  private void updateRobotPoses() {

    Pose2d testing = m_drive.getPoseMeters();

    m_field2d.setRobotPose(m_drive.getPoseMeters());

 
    m_field2d.getObject("CamCalcPose 1").setPose(m_visPosEst.visionMeasurement[0].toPose2d());

    m_field2d.getObject("CamPose 1").setPose(m_visPosEst.camPose[0].toPose2d());

     m_field2d.getObject("CamCalcPose 2").setPose(m_visPosEst.visionMeasurement[1].toPose2d());

     m_field2d.getObject("CamPose 2").setPose(m_visPosEst.camPose[1].toPose2d());

     m_field2d.getObject("Temp 1").setPose(m_visPosEst.temp[0]);
 
     m_field2d.getObject("Temp 2").setPose(m_visPosEst.temp[1]);

  }

  public void periodic() {

    updateRobotPoses();

    if (RobotBase.isSimulation())

      simulationPeriodic();

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void simulationPeriodic() {
  }
}
