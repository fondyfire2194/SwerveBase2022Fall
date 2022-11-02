/*
 *******************************************************************************************
 * Copyright (C) 2022 FRC Team 1736 Robot Casserole - www.robotcasserole.org
 *******************************************************************************************
 *
 * This software is released under the MIT Licence - see the license.txt
 *  file in the root of this repo.
 *
 * Non-legally-binding statement from Team 1736:
 *  Thank you for taking the time to read through our software! We hope you
 *   find it educational and informative! 
 *  Please feel free to snag our software for your own use in whatever project
 *   you have going on right now! We'd love to be able to help out! Shoot us 
 *   any questions you may have, all our contact info should be on our website
 *   (listed above).
 *  If you happen to end up using our software to make money, that is wonderful!
 *   Robot Casserole is always looking for more sponsors, so we'd be very appreciative
 *   if you would consider donating to our club to help further STEM education.
 */

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;

public class LEDControllerI2C {

    private static LEDControllerI2C ledCtrl = null;

    public static I2C arduino; // create initial I2C object, assign name.

    public int currPattern;

    public static synchronized LEDControllerI2C getInstance() {

        arduino = new I2C(I2C.Port.kMXP, 8); // when robot turns on, assign arduino to the onboard I2C bus, and
                                             // assign it port #8.

        if (ledCtrl == null)
            ledCtrl = new LEDControllerI2C();
        return ledCtrl;
    }

    public enum LEDPatterns {
        RedColorSparkle, // Red Color Sparkle
        YellowColorSparkle, // Yellow Color Sparkle
        GreenAlert, // Green Alert
        RedFade, // Red Fade
        CasseroleStripes, // Rainbow Fade Chase
        DisabledPattern; // Fire or something like that, we can't control it

        // Here is the UpdateLEDs function. This passes the string to the Ard/RIOduino.

    }

    // This is the private constructor that will be called once by getInstance() and
    // it
    // should instantiate anything that will be required by the class
    private LEDControllerI2C() {

        Thread monitorThread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    while (!Thread.currentThread().isInterrupted()) {
                        ledUpdater();
                        Thread.sleep(100);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });

        // Set up thread properties and start it off
        monitorThread.setName("CasseroleLEDThread");
        monitorThread.setPriority(Thread.MIN_PRIORITY);
        monitorThread.start();
    }

    public void ledUpdater() {

        double matchTime = DriverStation.getMatchTime();

        currPattern = getOrdinal(LEDPatterns.DisabledPattern);

        if (matchTime <= 30 && matchTime >= 5 && DriverStation.isTeleop()) {

            currPattern = getOrdinal(LEDPatterns.YellowColorSparkle);

        } else {

            if (DriverStation.isTeleopEnabled() & DriverStation.getAlliance() != Alliance.Blue)

                currPattern = getOrdinal(LEDPatterns.RedFade);

            if (DriverStation.isTeleopEnabled() & DriverStation.getAlliance() == Alliance.Blue)

                currPattern = getOrdinal(LEDPatterns.GreenAlert);

        }

        updateLEDs((short) currPattern); // Passing DISABLED string to UpdateLEDs

    }

    public int getOrdinal(LEDPatterns patt) {
        return patt.ordinal();
    }

    private static byte[] shortToBytes(final short data) {
        return new byte[] {
                // (byte) ((data >> 24) & 0xff),
                // (byte) ((data >> 16) & 0xff),
                (byte) ((data >> 8) & 0xff),
                (byte) ((data >> 0) & 0xff),
        };
    }

    public void updateLEDs(short n) // Constructor, pass it an int argument.

    {
        boolean t = arduino.writeBulk(shortToBytes(n));

    }

}
