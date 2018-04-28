package drivers;

import edu.wpi.first.wpilibj.Joystick;
import robot.RobotConstants;

public class Driver {
    
    public Joystick leftJoy;
    public Joystick rightJoy;
    
    public Driver() {
        leftJoy = new Joystick(RobotConstants.LEFT_JOYSTICK_PORT);
        rightJoy = new Joystick(RobotConstants.RIGHT_JOYSTICK_PORT);
    }
    
}
