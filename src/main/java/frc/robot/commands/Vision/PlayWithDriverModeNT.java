// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PlayWithDriverModeNT extends CommandBase {
  /** Creates a new PlayWithDriverMode. */
  private PhotonCamera m_cam;
  private boolean m_on;
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;

  public PlayWithDriverModeNT(PhotonCamera cam, boolean on) {
    // Use addRequirements() here to declare subsystem dependencies. m_cam=cam;
    m_on = on;
    m_cam = cam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // Get the table within that instance that contains the data. There can
    // be as many tables as you like and exist to make it easier to organize
    // your data. In this case, it's a table called datatable.
    NetworkTable table = inst.getTable("photonvision/picam");

    // Get the entries within that table that correspond to the X and Y values
    // for some operation in your program.
    xEntry = table.getEntry("driverMode");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("TSTNTy", xEntry.getBoolean(false));
    xEntry.setBoolean(m_on);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xEntry.getBoolean(false) == m_on;
  }
}
