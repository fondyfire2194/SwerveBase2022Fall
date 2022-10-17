// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class SimVisionSystem extends SubsystemBase {
  /** Creates a new SimVision. */// Simulated Vision System.
// Configure these to match your PhotonVision Camera,
// pipeline, and LED setup.
double camDiagFOV = 170.0; // degrees - assume wide-angle camera
double camPitch = Units.radiansToDegrees(0); // degrees
double camHeightOffGround = 1; // meters
double maxLEDRange = 20; // meters
int camResolutionWidth = 640; // pixels
int camResolutionHeight = 480; // pixels
double minTargetArea = 10; // square pixels
  public SimVisionSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}






