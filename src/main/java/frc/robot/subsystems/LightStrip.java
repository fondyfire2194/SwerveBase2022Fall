// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;//do driverstation stuff in Robot.java 
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class LightStrip extends SubsystemBase {

  /**
   * Creates a new Lights.
   */

  // public AddressableLED m_led;
  // public AddressableLEDBuffer m_ledBuffer;
  public int m_numOfLEDs;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  public boolean allianceColorSet;

  int red;
  int green;
  int blue;

  public Timer m_timer;

  private int loopCtr;

  private int ledCtr;

  private int ledCtrLast;

  public LightStrip(int port, int numOfLEDs) {
    m_led = new AddressableLED(port);
    m_numOfLEDs = numOfLEDs;
    m_ledBuffer = new AddressableLEDBuffer(numOfLEDs);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    m_timer = new Timer();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  public void forceAllianceColor(boolean blueAlliance) {
    allianceColorSet = false;
    if (blueAlliance) {
      red = 0;
      green = 0;
      blue = 255;
    } else {
      red = 255;
      green = 0;
      blue = 0;
    }

    for (int i = 0; i < m_numOfLEDs; i++) {
      SmartDashboard.putNumber("LEDI", i);
      SmartDashboard.putNumber("LEDCol ", red);

      m_ledBuffer.setRGB(i, red, green, blue);

      m_led.setData(m_ledBuffer);
    }

    allianceColorSet = true;

  }

  // methods for different lights patterns
  public void lightsaber(boolean blueStart)// for climber arm

  {
    if (ledCtr == 0) {
      ledCtrLast = -1;
      if (blueStart) {
        red = 0;
        green = 0;
        blue = 255;
      } else {
        red = 255;
        green = 0;
        blue = 0;
      }
    }

    loopCtr++;

    if (loopCtr > 25) {

      ledCtr++;

      loopCtr = 0;
    }

    if (ledCtr < m_numOfLEDs && ledCtr != ledCtrLast) {

      SmartDashboard.putNumber("LEDI", ledCtr);

      m_ledBuffer.setRGB(ledCtr, 255, 0, 0);

      m_led.setData(m_ledBuffer);

      ledCtrLast = ledCtr;
    }
  }

}