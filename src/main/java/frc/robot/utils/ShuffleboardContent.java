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

                ShuffleboardTab x = Shuffleboard.getTab("Drivetrain");

                x.addBoolean("DriveCAN" + String.valueOf(m_moduleNumber), () -> m_sm.driveMotorConnected)
                                .withPosition(8, m_moduleNumber);
                x.addBoolean("TurnCAN" + String.valueOf(m_moduleNumber), () -> m_sm.turnMotorConnected)
                                .withPosition(9, m_moduleNumber);

        }

        public static void initDriveShuffleboard(SwerveModuleSparkMax m_sm) {
                ModulePosition m_modulePosition = m_sm.getModulePosition();
                int m_moduleNumber = m_modulePosition.ordinal();
                String driveLayout = m_modulePosition.toString() + " Drive";
                ShuffleboardLayout drLayout = Shuffleboard.getTab("Drivetrain")
                                .getLayout(driveLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                drLayout.addNumber("Drive Speed MPS " + m_modulePosition.toString(), () -> m_sm.getDriveVelocity());

                drLayout.addNumber("Drive Position " + m_modulePosition.toString(), () -> m_sm.getDrivePosition());

                drLayout.addNumber("App Output " + m_modulePosition.toString(),
                                () -> m_sm.m_driveMotor.getAppliedOutput());

                drLayout.addNumber("Current Amps " + m_modulePosition.toString(),
                                () -> m_sm.getDriveCurrent());

                drLayout.addNumber("Firmware" + m_modulePosition.toString(),
                                () -> m_sm.m_driveMotor.getFirmwareVersion());

        }

        public static void initTurnShuffleboard(SwerveModuleSparkMax m_sm) {
                ModulePosition m_modulePosition = m_sm.getModulePosition();
                int m_moduleNumber = m_modulePosition.ordinal();

                String turnLayout = m_modulePosition.toString() + " Turn";

                ShuffleboardLayout tuLayout = Shuffleboard.getTab("Drivetrain")
                                .getLayout(turnLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 2)
                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                tuLayout.addNumber("Turn Setpoint Deg " + m_modulePosition.toString(), () -> m_sm.angle);

                tuLayout.addNumber("Turn Enc Pos " + m_modulePosition.toString(),
                                () -> m_sm.getTurnPosition() % 360);

                tuLayout.addNumber("Act Ang Deg " + m_modulePosition.toString(),
                                () -> m_sm.actualAngleDegrees);

                tuLayout.addNumber("TurnAngleOut" + m_modulePosition.toString(),
                                () -> m_sm.m_turningMotor.getAppliedOutput());

                tuLayout.addNumber("Position" + m_modulePosition.toString(), () -> m_sm.m_turnCANcoder.getMyPosition());

                tuLayout.addNumber("Current Amps" + m_modulePosition.toString(), () -> m_sm.getTurnCurrent());

                tuLayout.addNumber("Abs Offset" + m_modulePosition.toString(), () -> m_sm.m_turningEncoderOffset);

                tuLayout.addNumber("Firmware" + m_modulePosition.toString(),
                                () -> m_sm.m_turningMotor.getFirmwareVersion());

        }

        public static void initCoderBooleanShuffleboard(SwerveModuleSparkMax m_sm) {

                ModulePosition m_modulePosition = m_sm.getModulePosition();
                int m_moduleNumber = m_modulePosition.ordinal();

                ShuffleboardTab x = Shuffleboard.getTab("CanCoders");

                x.addBoolean("CANOK" + String.valueOf(m_moduleNumber), () -> m_sm.turnCoderConnected)
                                .withPosition(8, m_moduleNumber);
                x.addBoolean("Fault" + String.valueOf(m_moduleNumber), () -> m_sm.m_turnCANcoder.getFaulted())
                                .withPosition(9, m_moduleNumber);

        }

        public static void initCANCoderShuffleboard(SwerveModuleSparkMax m_sm) {
                ModulePosition m_modulePosition = m_sm.getModulePosition();
                int m_moduleNumber = m_modulePosition.ordinal();

                String canCoderLayout = m_modulePosition.toString() + " CanCoder";

                ShuffleboardLayout coderLayout = Shuffleboard.getTab("CanCoders")
                                .getLayout(canCoderLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 0)
                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                coderLayout.addNumber("Position" + m_modulePosition.toString(),
                                () -> m_sm.m_turnCANcoder.getMyPosition());
                coderLayout.addNumber("Abs Position" + m_modulePosition.toString(),
                                () -> m_sm.m_turnCANcoder.getAbsolutePosition());
                coderLayout.addNumber("Velocity" + m_modulePosition.toString(),
                                () -> m_sm.m_turnCANcoder.getVelValue());
                coderLayout.addString(" MagField " + m_modulePosition.toString(),
                                () -> m_sm.m_turnCANcoder.getMagnetFieldStrength().toString());
                coderLayout.addNumber("Battery Volts" + m_modulePosition.toString(),
                                () -> m_sm.m_turnCANcoder.getBatValue());
                coderLayout.addNumber("Bus Volts" + m_modulePosition.toString(),
                                () -> m_sm.m_turnCANcoder.getBusVoltage());

                coderLayout.addNumber("Abs Offset" + m_modulePosition.toString(), () -> m_sm.m_turningEncoderOffset);

                coderLayout.addNumber("Firmware#" + m_modulePosition.toString(),
                                () -> m_sm.m_turnCANcoder.getFirmwareVersion());

        }

        public void initTuningShuffleboard(SwerveModuleSparkMax m_sm) {
                String tuneLayout = m_sm.m_modulePosition.toString() + " Tuning";
                ShuffleboardLayout tuLayout = Shuffleboard.getTab("Drivetrain")
                                .getLayout(tuneLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 2)
                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                tuLayout.addNumber("Turn Setpoint Deg " + m_modulePosition.toString(), () -> m_sm.angle);

                tuLayout.addNumber("Turn Enc Pos " + m_modulePosition.toString(),
                                () -> m_sm.getTurnPosition());

                tuLayout.addNumber("Act Ang Deg " + m_modulePosition.toString(),
                                () -> m_sm.actualAngleDegrees);

                tuLayout.addNumber("TurnAngleOut" + m_modulePosition.toString(),
                                () -> m_sm.m_turningMotor.getAppliedOutput());

                tuLayout.addNumber("Position" + m_modulePosition.toString(), () -> m_sm.m_turnCANcoder.getMyPosition());

                tuLayout.addNumber("Abs Offset" + m_modulePosition.toString(), () -> m_sm.m_turningEncoderOffset);

                tuLayout.addString("Fault" + m_modulePosition.toString(),
                                () -> m_sm.m_turnCANcoder.getFaulted() ? "TRUE" : "FALSE");

        }

}
