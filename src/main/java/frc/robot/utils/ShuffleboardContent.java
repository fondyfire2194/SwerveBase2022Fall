// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModuleSparkMax;

/** Add your docs here. */
public class ShuffleboardContent {

        private int m_moduleNumber;

        private ModulePosition m_modulePosition;

        static ShuffleboardLayout boolsLayout;

        public ShuffleboardContent() {

        }

        public static void initBooleanShuffleboard(SwerveModuleSparkMax m_sm) {

                ModulePosition m_modulePosition = m_sm.getModulePosition();
                int m_moduleNumber = m_modulePosition.ordinal();
                String abrev = m_sm.modAbrev[m_moduleNumber];

                ShuffleboardTab x = Shuffleboard.getTab("Drivetrain");

                x.addBoolean("DriveCAN" + abrev, () -> m_sm.driveMotorConnected)
                                .withPosition(8, m_moduleNumber);
                x.addBoolean("TurnCAN" + abrev, () -> m_sm.turnMotorConnected)
                                .withPosition(9, m_moduleNumber);

        }

        public static void initDriveShuffleboard(SwerveModuleSparkMax m_sm) {
                ModulePosition m_modulePosition = m_sm.getModulePosition();
                int m_moduleNumber = m_modulePosition.ordinal();
                String abrev = m_sm.modAbrev[m_moduleNumber];
                String driveLayout = m_modulePosition.toString() + " Drive";
                ShuffleboardLayout drLayout = Shuffleboard.getTab("Drivetrain")
                                .getLayout(driveLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                drLayout.addNumber("Drive Speed MPS " + abrev, () -> m_sm.getDriveVelocity());

                drLayout.addNumber("Drive Position " + abrev, () -> m_sm.getDrivePosition());

                drLayout.addNumber("App Output " + abrev,
                                () -> m_sm.m_driveMotor.getAppliedOutput());

                drLayout.addNumber("Current Amps " + abrev,
                                () -> m_sm.getDriveCurrent());

                drLayout.addNumber("Firmware" + abrev,
                                () -> m_sm.m_driveMotor.getFirmwareVersion());

        }

        public static void initTurnShuffleboard(SwerveModuleSparkMax m_sm) {
                ModulePosition m_modulePosition = m_sm.getModulePosition();
                int m_moduleNumber = m_modulePosition.ordinal();
                String abrev = m_sm.modAbrev[m_moduleNumber];
                String turnLayout = m_modulePosition.toString() + " Turn";

                ShuffleboardLayout tuLayout = Shuffleboard.getTab("Drivetrain")
                                .getLayout(turnLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 2)
                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                tuLayout.addNumber("Turn Setpoint Deg " + abrev, () -> m_sm.angle);

                tuLayout.addNumber("Turn Enc Pos " + abrev,
                                () -> m_sm.getTurnPosition() % 360);

                tuLayout.addNumber("Act Ang Deg " + abrev,
                                () -> m_sm.actualAngleDegrees);

                tuLayout.addNumber("TurnAngleOut" + abrev,
                                () -> m_sm.m_turningMotor.getAppliedOutput());

                tuLayout.addNumber("Position" + abrev, () -> m_sm.m_turnCANcoder.getMyPosition());

                tuLayout.addNumber("Current Amps" + abrev, () -> m_sm.getTurnCurrent());

                tuLayout.addNumber("Abs Offset" + abrev, () -> m_sm.m_turningEncoderOffset);

                tuLayout.addNumber("Firmware" + abrev,
                                () -> m_sm.m_turningMotor.getFirmwareVersion());

        }

        public static void initCoderBooleanShuffleboard(SwerveModuleSparkMax m_sm) {

                ModulePosition m_modulePosition = m_sm.getModulePosition();
                int m_moduleNumber = m_modulePosition.ordinal();
                String abrev = m_sm.modAbrev[m_moduleNumber];
                ShuffleboardTab x = Shuffleboard.getTab("CanCoders");

                x.addBoolean("CANOK" + abrev, () -> m_sm.turnCoderConnected)
                                .withPosition(8, m_moduleNumber);
                x.addBoolean("Fault" + abrev, () -> m_sm.m_turnCANcoder.getFaulted())
                                .withPosition(9, m_moduleNumber);

        }

        public static void initCANCoderShuffleboard(SwerveModuleSparkMax m_sm) {
                ModulePosition m_modulePosition = m_sm.getModulePosition();
                int m_moduleNumber = m_modulePosition.ordinal();
                String abrev = m_sm.modAbrev[m_moduleNumber];
                String canCoderLayout = m_modulePosition.toString() + " CanCoder";

                ShuffleboardLayout coderLayout = Shuffleboard.getTab("CanCoders")
                                .getLayout(canCoderLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 0)
                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                coderLayout.addNumber("Position" + abrev,
                                () -> m_sm.m_turnCANcoder.getMyPosition());
                coderLayout.addNumber("Abs Position" + abrev,
                                () -> m_sm.m_turnCANcoder.getAbsolutePosition());
                coderLayout.addNumber("Velocity" + abrev,
                                () -> m_sm.m_turnCANcoder.getVelValue());
                coderLayout.addString(" MagField " + abrev,
                                () -> m_sm.m_turnCANcoder.getMagnetFieldStrength().toString());
                coderLayout.addNumber("Battery Volts" + abrev,
                                () -> m_sm.m_turnCANcoder.getBatValue());
                coderLayout.addNumber("Bus Volts" + abrev,
                                () -> m_sm.m_turnCANcoder.getBusVoltage());

                coderLayout.addNumber("Abs Offset" + abrev, () -> m_sm.m_turningEncoderOffset);

                coderLayout.addNumber("Firmware#" + abrev,
                                () -> m_sm.m_turnCANcoder.getFirmwareVersion());

        }

        public static void initMisc(DriveSubsystem drive) {

                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Drivetrain");

                drLayout1.addBoolean("FieldOr", () -> drive.m_fieldOriented).withPosition(8,4)

                                .withSize(1, 1);
                drLayout1.addNumber("GyroYaw", () -> drive.getHeadingDegrees()).withPosition(9,4)

                                .withSize(1, 1);
        }
}
