package frc.team3128.common.hardware.input;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Wrapper for the WPILib Joystick class. Works with Logitech Extreme 3D Pro and Thrustmaster T16000M.
 * 
 */

public class NAR_XboxController extends XboxController {

    private XboxController controller;

    private JoystickButton buttons[] = new JoystickButton[10];   

    public NAR_XboxController(int port) {
        super(port);

        controller = new XboxController(port);

        for (int i = 0; i < 10; i++) {
            buttons[i] = new JoystickButton(controller, i + 1);
        }   
    }

    public JoystickButton getXboxButton(int buttonNum) {
        return buttons[buttonNum - 1];
    }

}