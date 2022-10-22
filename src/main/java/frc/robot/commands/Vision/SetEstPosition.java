// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import org.ejml.dense.block.MatrixOps_FDRB;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionPoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetEstPosition extends InstantCommand {
  private final DriveSubsystem m_drive;
  private double m_xValue;
  private double m_yValue;
  private double m_yaw;

  public SetEstPosition(DriveSubsystem drive, double x, double y, double yaw) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_xValue = x;
    m_yValue = y;
    m_yaw = yaw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    var poseMeters = new Pose2d(m_xValue, m_yValue, new Rotation2d(0, m_yaw));

    m_drive.resetModuleEncoders();

    m_drive.m_odometry.resetPosition(poseMeters, m_drive.getHeadingRotation2d());
  }
}
