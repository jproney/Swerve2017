package subsystems;

import com.ctre.PigeonImu;

import commands.SwerveControlCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import mathUtil.MathUtil;
import mathUtil.SwerveCalculator;
import mathUtil.Vector2d;
import robot.Robot;
import robot.RobotConstants;

public class DriveTrain extends Subsystem {

    private SwerveModule fl;
    private SwerveModule fr;
    private SwerveModule bl;
    private SwerveModule br;
    private PigeonImu gyro;

    public DriveTrain() {
        fl = new SwerveModule(SwerveModule.ModuleID.FL);
        fr = new SwerveModule(SwerveModule.ModuleID.FR);
        bl = new SwerveModule(SwerveModule.ModuleID.BL);
        fr = new SwerveModule(SwerveModule.ModuleID.BR);
        gyro = new PigeonImu(RobotConstants.GYRO_PORT);
    }

    public void resetGyro() {
        gyro.SetYaw(0.0);
    }

    public double getGyroHeading() {
        double[] ypr = new double[3];
        gyro.GetYawPitchRoll(ypr);
        return MathUtil.wrapAngleRad(ypr[0]);
    }

    public void swerve(double rot, Vector2d trans) {
        Vector2d[] swerveVecs = SwerveCalculator.calculateAllModules(getGyroHeading(), rot, trans, fl.getPosVec(),
                fr.getPosVec(), bl.getPosVec(), br.getPosVec());
        fl.setToVectorSmart(swerveVecs[0]);
        fr.setToVectorSmart(swerveVecs[1]);
        bl.setToVectorSmart(swerveVecs[2]);
        br.setToVectorSmart(swerveVecs[3]);
    }

    public void stop() {
        fl.setDrivePower(0.0);
        fr.setDrivePower(0.0);
        bl.setDrivePower(0.0);
        br.setDrivePower(0.0);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new SwerveControlCommand(Robot.runningrobot.driver.leftJoy,
                Robot.runningrobot.driver.rightJoy));
    }

}
