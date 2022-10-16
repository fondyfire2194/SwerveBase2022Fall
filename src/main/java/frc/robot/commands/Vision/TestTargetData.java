// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import org.opencv.core.TickMeter;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.AprilTagData;

public class TestTargetData extends CommandBase {
  /** Creates a new TestTargetData. */
  private int n;
  private double testTimer;

  public TestTargetData() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    n = 0;
    testTimer = Timer.getFPGATimestamp();
    // AprilTagData.highestTagNumber = (int)
    // AprilTagData.tagLocationData[AprilTagData.tagLocationData.length - 1][0];
    SmartDashboard.putNumber("HighTag", AprilTagData.highestTagNumber);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Location", AprilTagData.getTagLocation(n));
    SmartDashboard.putNumber("N", n);
    Transform3d temp = AprilTagData.getTransform3d(n);
    double x = temp.getX();
    SmartDashboard.putNumber("XData", x);

    if (Timer.getFPGATimestamp() > testTimer + 1) {
      n++;
      testTimer = Timer.getFPGATimestamp();
    }
    if (n >= AprilTagData.highestTagNumber)
      n = 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
