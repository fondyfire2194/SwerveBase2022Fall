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

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;

public class LEDController {

    private static LEDController ledCtrl = null;

    PWM ctrl;

    public static synchronized LEDController getInstance() {

        if (ledCtrl == null)
            ledCtrl = new LEDController();
        return ledCtrl;
    }

    public enum LEDPatterns {
        RedColorSparkle(0, 1.0), // Red Color Sparkle
        YellowColorSparkle(1, 1.3), // Yellow Color Sparkle
        GreenAlert(4, 1.5), // Green Alert
        RedFade(5, 1.7), // Red Fade
        CasseroleStripes(6, 2.0), // Rainbow Fade Chase
        DisabledPattern(-1, 0.0); // Fire or something like that, we can't control it

        public final int value;
        public final double period_ms;

        private LEDPatterns(int value, double period_ms) {
            this.value = value;
            this.period_ms = period_ms;
        }

        public int toInt() {
            return this.value;
        }

        public double getPeriod() {
            return this.period_ms;
        }
    }

    // This is the private constructor that will be called once by getInstance() and
    // it
    // should instantiate anything that will be required by the class
    private LEDController() {

        ctrl = new PWM(Constants.LED_CONTROLLER_PORT);

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

        LEDPatterns curPattern = LEDPatterns.DisabledPattern;

        if (matchTime <= 30 && matchTime >= 5 && DriverStation.isTeleop()) {

            curPattern = LEDPatterns.YellowColorSparkle;

        } else {

            if (DriverStation.isTeleopEnabled() & DriverStation.getAlliance() != Alliance.Blue)

                curPattern = LEDPatterns.RedFade;

            if (DriverStation.isTeleopEnabled() & DriverStation.getAlliance() == Alliance.Blue)

                curPattern = LEDPatterns.GreenAlert;

        }

        ctrl.setSpeed((curPattern.getPeriod() - 1.5) * 2.0);

    }
}