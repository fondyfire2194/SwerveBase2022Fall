// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Sim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/** Add your docs here. */
public class CANEncoderSim {
    private final String deviceKey;
    private final String positionKey = "Position";
    private final String velocityKey = "Velocity";
    private final SimDouble positionProp;
    private final SimDouble velocityProp;
    private final SimDeviceSim simSpark;
    private final double sign;
    public CANEncoderSim(int id, boolean invert) {
      deviceKey = "SPARK MAX [" + id + "]";
      simSpark = new SimDeviceSim(deviceKey);
      positionProp = simSpark.getDouble(positionKey);
      velocityProp = simSpark.getDouble(velocityKey);
      sign = invert ? -1.0 : 1.0;
    }
    public void setPosition(double position) {positionProp.set(sign*position);}
    public void setVelocity(double velocity) {velocityProp.set(sign*velocity);}
  }

