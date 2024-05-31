package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class JoystickContainer {

    private XboxController driverJoystick;
    private XboxController operatorJoystick;

    JoystickContainer(XboxController driveJoy, XboxController opJoy) {
        driverJoystick = driveJoy;
        operatorJoystick = opJoy;
    }

    public JoystickButton driveButton(int buttonId) {
        return new JoystickButton(driverJoystick, buttonId);
    }
    public JoystickButton opButton(int buttonId) {
        return new JoystickButton(operatorJoystick, buttonId);
    }

    public double driveRawAxis(int axisId) {
        return driverJoystick.getRawAxis(axisId);
    }

     public double opRawAxis(int axisId) {
        return operatorJoystick.getRawAxis(axisId);
    }


    public double driveAxis(int axisId) {
        final double DEADZONE = .05;
        double rawAxis = driverJoystick.getRawAxis(axisId);
        if (Math.abs(rawAxis) < DEADZONE) {
            return 0.0;
        } else {
            return rawAxis;
        }
    }

    public double operatorAxis(int axisId) {
        final double DEADZONE = .05;
        double rawAxis = operatorJoystick.getRawAxis(axisId);
        if (Math.abs(rawAxis) < DEADZONE) {
            return 0.0;
        } else {
            return rawAxis;
        }
    }

}

