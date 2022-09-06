// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.ctre.phoenix.sensors.MagnetFieldStrength;

import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class CTRECanCoder extends WPI_CANCoder {
    CANCoderFaults faults = new CANCoderFaults();
    CANCoderStickyFaults stickyFaults = new CANCoderStickyFaults();

    public CTRECanCoder(int deviceNumber) {
        super(deviceNumber);
        // TODO Auto-generated constructor stub
    }

    public void init(String name, String configName) {

        CANCoderConfiguration m_canCoderConfiguration = new CANCoderConfiguration();

        m_canCoderConfiguration.unitString = configName;

        this.configAllSettings(m_canCoderConfiguration);

    }

    public String[] getStickyFaults() {
        String[] temp = new String[4];
        temp[0] = stickyFaults.HardwareFault ? "True " : "False";
        temp[1] = stickyFaults.UnderVoltage ? "True " : "False";
        temp[2] = stickyFaults.ResetDuringEn ? "True " : "False";
        temp[3] = stickyFaults.APIError ? "True " : "False";
        return temp;
    }

    public String[] getFaults() {
        String[] temp = new String[4];

        return temp;

    }

    public boolean getFaulted() {
        return faults.hasAnyFault() || stickyFaults.hasAnyFault();

    }

    public double getMyPosition() {
        return this.getPosition();
    }

    public String getPosUnits() {
        return this.getLastUnitString();
    }

    public double getPosTstmp() {

        return this.getLastTimestamp();
    }

    public double getAbsValue() {
        return this.getAbsolutePosition();

    }

    public String getAbsUnits() {

        return this.getLastUnitString();

    }

    public double getAbsTstmp() {
        return this.getLastTimestamp();
    }

    public double getVelValue() {
        return this.getVelocity();
    }

    public String getVelUnits() {
        return this.getLastUnitString();
    }

    public double getVelTstmp() {
        return this.getLastTimestamp();
    }

    public double getBatValue() {
        return this.getBusVoltage();
    }

    public String getBatUnits() {
        return this.getLastUnitString();
    }

    public double getBatTstmp() {
        return this.getLastTimestamp();
    }

    /* Report miscellaneous attributes about the CANCoder */
    public MagnetFieldStrength getMagnetStrength() {
        return this.getMagnetFieldStrength();
    }

    public String getMagnetStrengthUnits() {
        return this.getLastUnitString();
    }

    public double magnetStrengthTstmp() {
        return this.getLastTimestamp();
    }

}
