// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModuleSparkMax;

/** Add your docs here. */
public class SwerveShuffleboard {

    private int row = 0;
    private int col = 0;

    public SwerveShuffleboard(DriveSubsystem drive, SwerveModuleSparkMax smod) {

        ShuffleboardLayout absenc1 = Shuffleboard.getTab("SwerveModules")
                .getLayout("i", BuiltInLayouts.kList).withPosition(0, 0)
                .withSize(2, 5).withProperties(Map.of("Label position", "TOP"));

       // absenc1.addString("FrontLeft", () -> drive.m_frontLeft.getState().toString());

    }

}