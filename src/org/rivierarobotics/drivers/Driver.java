package org.rivierarobotics.drivers;

import org.rivierarobotics.commands.SetModulePosition;
import org.rivierarobotics.commands.SetModulePower;
import org.rivierarobotics.robot.RobotConstants;
import org.rivierarobotics.subsystems.SwerveModule.ModuleID;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Driver {
    
    public Joystick leftJoy;
    public Joystick rightJoy;
    public Joystick buttons;
    
    public Driver() {
        leftJoy = new Joystick(RobotConstants.LEFT_JOYSTICK_PORT);
        rightJoy = new Joystick(RobotConstants.RIGHT_JOYSTICK_PORT);
        buttons = new Joystick(RobotConstants.BUTTONS_PORT);
        
        JoystickButton setPower1 = new JoystickButton(leftJoy,6);
        setPower1.whenPressed(new SetModulePosition(5000, ModuleID.FL));
        setPower1.whenReleased(new SetModulePosition(0, ModuleID.FL));
        
        JoystickButton setPower2 = new JoystickButton(leftJoy,5);
        setPower2.whenPressed(new SetModulePosition(5000, ModuleID.BR));
        setPower2.whenReleased(new SetModulePosition(0,ModuleID.BR));
    }
    
}
