// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Add your docs here. */
public class GamepadButtons {
    public JoystickButton X_button;
    public JoystickButton Y_button;
    public JoystickButton A_button;
    public JoystickButton B_button;

    public JoystickButton leftTrigger;
    public JoystickButton rightTrigger;

    public JoystickButton leftBumper;
    public JoystickButton rightBumper;

    public JoystickButton back;
    public JoystickButton start;

    public JoystickButton leftStick;
    public JoystickButton rightStick;

    public POVButton upButton;
    public POVButton rightButton;
    public POVButton downButton;
    public POVButton leftButton;

    public GamepadButtons(XboxController controller, boolean model) {
        // Setup gamepad LOGITECH

        if (!model) {
            X_button = new JoystickButton(controller, 1);
            A_button = new JoystickButton(controller, 2);
            B_button = new JoystickButton(controller, 3);
            Y_button = new JoystickButton(controller, 4);
        }
        else{
            X_button = new JoystickButton(controller, 1);
            A_button = new JoystickButton(controller, 2);
            B_button = new JoystickButton(controller, 3);
            Y_button = new JoystickButton(controller, 4);
        }

        
        leftTrigger = new JoystickButton(controller, 5);
        rightTrigger = new JoystickButton(controller, 6);

        leftBumper = new JoystickButton(controller, 7);
        rightBumper = new JoystickButton(controller, 8);


        back = new JoystickButton(controller, 9);
        start = new JoystickButton(controller, 10);

        leftStick = new JoystickButton(controller, 11);
        rightStick = new JoystickButton(controller, 12);

        upButton = new POVButton(controller, 0);
        rightButton = new POVButton(controller, 90);
        downButton = new POVButton(controller, 180);
        leftButton = new POVButton(controller, 270);

    }

}
