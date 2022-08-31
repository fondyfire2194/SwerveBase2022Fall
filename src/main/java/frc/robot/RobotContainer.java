// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SetSwerveDrive;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  final DriveSubsystem m_robotDrive = new DriveSubsystem();

  public final FieldSim m_fieldSim = new FieldSim(m_robotDrive);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_fieldSim.initSim();

    SmartDashboard.putData("ResetPose",
        new SetSwerveOdometry(m_robotDrive, m_fieldSim, new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new SetSwerveDrive(
            m_robotDrive, () -> -m_driverController.getRawAxis(1),
            () -> m_driverController.getRawAxis(0),
            () ->  m_driverController.getRawAxis(2),
            true,
            true));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // // public Command getAutonomousCommand() {
  // // // Create config for trajectory
  // // TrajectoryConfig config =
  // // new TrajectoryConfig(
  // // AutoConstants.kMaxSpeedMetersPerSecond,
  // // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  // // // Add kinematics to ensure max speed is actually obeyed
  // // .setKinematics(m_robotDrive.kSwerveKinematics);

  // // // An example trajectory to follow. All units in meters.
  // // Trajectory exampleTrajectory =
  // // TrajectoryGenerator.generateTrajectory(
  // // // Start at the origin facing the +X direction
  // // new Pose2d(0, 0, new Rotation2d(0)),
  // // // Pass through these two interior waypoints, making an 's' curve path
  // // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  // // // End 3 meters straight ahead of where we started, facing forward
  // // new Pose2d(3, 0, new Rotation2d(0)),
  // // config);

  // // var thetaController =
  // // new ProfiledPIDController(
  // // AutoConstants.kPThetaController, 0, 0,
  // AutoConstants.kThetaControllerConstraints);
  // // thetaController.enableContinuousInput(-Math.PI, Math.PI);

  // // SwerveControllerCommand swerveControllerCommand =
  // // new SwerveControllerCommand(x,
  // // exampleTrajectory,
  // // m_robotDrive::getPose, // Functional interface to feed supplier
  // // m_robotDrive.kSwerveKinematics,

  // // // Position controllers
  // // new PIDController(AutoConstants.kPXController, 0, 0),
  // // new PIDController(AutoConstants.kPYController, 0, 0),
  // // thetaController,
  // // m_robotDrive::setModuleStates,
  // // m_robotDrive);

  // // Reset odometry to the starting pose of the trajectory.
  // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  // // Run path following command, then stop at the end.
  // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
  // false,false));
  // }
}
