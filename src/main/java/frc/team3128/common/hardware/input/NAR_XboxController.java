package frc.team3128.common.hardware.input;

import java.util.HashMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class NAR_XboxController extends XboxController {

    private XboxController controller;

    private String buttonNames[] = {
        "A",
        "B",
        "X",
        "Y",
        "LeftBumper",
        "RightBumper",
        "Back",
        "Start",
        "LeftStick",
        "RightStick"
    };
    
    private HashMap<String, JoystickButton> buttons;

    public NAR_XboxController(int port) {
        super(port);

        controller = new XboxController(port);

        for (int i = 0; i < 10; i++) {
            buttons.put(buttonNames[i], new JoystickButton(controller, i));
        }   
    }

    public JoystickButton getButton(String buttonName) {
        return buttons.get(buttonName);
    }

    public Trigger getLeftTrigger() {
        return new Trigger (() -> controller.getLeftTriggerAxis() >= 0.5);
    }

    public Trigger getRightTrigger() {
        return new Trigger (() -> controller.getRightTriggerAxis() >= 0.5);
    }

}