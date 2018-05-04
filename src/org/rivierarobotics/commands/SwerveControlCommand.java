package org.rivierarobotics.commands;

import org.rivierarobotics.mathUtil.MathUtil;
import org.rivierarobotics.mathUtil.Vector2d;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.SwerveModule.ModuleID;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

public class SwerveControlCommand extends Command{
    
    public static final Vector2d DEADBAND = new Vector2d(.2, .2);
    
    private DriveTrain dt;
    private Joystick transStick;
    private Joystick rotStick;
    
    public SwerveControlCommand(Joystick trans, Joystick spin) {
        dt = Robot.runningrobot.dt;
        transStick = trans;
        rotStick = spin;
        requires(dt);
    }
    
    @Override
    public void execute() {
        if(MathUtil.outOfDeadband(transStick, DEADBAND) || MathUtil.outOfDeadband(rotStick, DEADBAND)) {
              dt.getModule(ModuleID.FL).setPositionRads(rotStick.getDirectionRadians());
              dt.getModule(ModuleID.BR).setPositionRads(rotStick.getDirectionRadians());
//            Vector2d transVec = MathUtil.adjustDeadband(transStick, DEADBAND, true);
//            double spinVal = MathUtil.adjustDeadband(rotStick, DEADBAND, true).getX();
//            dt.swerve(spinVal, transVec);
        }
        else {
            dt.stop();
        }
    }

    
    @Override
    protected boolean isFinished() {
        return false;
    }

}