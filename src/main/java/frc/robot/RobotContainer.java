// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Vision.SetDriverMode;
import frc.robot.utils.ShuffleboardVision;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  

  Cameras cams;

  

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  // The driver's controller

  static Joystick leftJoystick = new Joystick(0);

  private XboxController m_coDriverController = new XboxController(1);

  final GamepadButtons driver = new GamepadButtons(m_coDriverController, true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Preferences.removeAll();
   
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    // Configure the button bindings

    cams = new Cameras();
    
    ShuffleboardVision.init(cams);
    
     

    SmartDashboard.putData(CommandScheduler.getInstance());

  SmartDashboard.putData("DMON",new SetDriverMode(cams.picam, true));
  SmartDashboard.putData("DMOFF", new SetDriverMode(cams.picam, false)); 
  
  SmartDashboard.putData("DMON", new SetDriverMode(cams.picam, true));
  SmartDashboard.putData("DMOFF", new SetDriverMode(cams.picam, false));



    JoystickButton button_8 = new JoystickButton(leftJoystick, 8);
    JoystickButton button_7 = new JoystickButton(leftJoystick, 7);

  
  }

 

 

  public double getThrottle() {
    return -leftJoystick.getThrottle();
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }

}
