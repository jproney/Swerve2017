package org.rivierarobotics.subsystems;

import org.rivierarobotics.mathUtil.MathUtil;
import org.rivierarobotics.mathUtil.Vector2d;
import org.rivierarobotics.robot.RobotConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class SwerveModule {

    public enum ModuleID {
        FR, FL, BL, BR;
    }

    private static final int MAX_POSSIBLE_VELOCITY = 100;
    private static final int MOTION_MAGIC_IDX = 0;
    private static final int SLOT_IDX = 0;
    private static final int TIMEOUT = 10;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.0;
    public static final double cruise = 0.0;
    public static final double accel = 0.0;

    public static final int STEERING_COUNTS_PER_REV = 4096 * 2;
    public static final double STEERING_ENC_ZERO_FL = 0.0;
    public static final double STEERING_ENC_ZERO_FR = 0.0;
    public static final double STEERING_ENC_ZERO_BL = 0.0;
    public static final double STEERING_ENC_ZERO_BR = 0.0;

    public double steeringEncZero;
    private Vector2d positionVec;
    private WPI_TalonSRX wheel;
    private WPI_TalonSRX steering;
    private ModuleID modID;
    private final double zeroPos;

    public SwerveModule(ModuleID id) {
        modID = id;
        switch (modID) {
            case FR:
                positionVec = new Vector2d(RobotConstants.ROBOT_WIDTH, RobotConstants.ROBOT_LENGTH);
                wheel = new WPI_TalonSRX(RobotConstants.FR_DRIVE);
                steering = new WPI_TalonSRX(RobotConstants.FR_STEERING);
                zeroPos = STEERING_ENC_ZERO_FL;
                break;
            case FL:
                positionVec = new Vector2d(-RobotConstants.ROBOT_WIDTH, RobotConstants.ROBOT_LENGTH);
                wheel = new WPI_TalonSRX(RobotConstants.FL_DRIVE);
                steering = new WPI_TalonSRX(RobotConstants.FL_STEERING);
                zeroPos = STEERING_ENC_ZERO_FR;
                break;
            case BR:
                positionVec = new Vector2d(RobotConstants.ROBOT_WIDTH, -RobotConstants.ROBOT_LENGTH);
                wheel = new WPI_TalonSRX(RobotConstants.FR_DRIVE);
                steering = new WPI_TalonSRX(RobotConstants.FR_STEERING);
                zeroPos = STEERING_ENC_ZERO_BL;
                break;
            case BL:
            default:
                positionVec = new Vector2d(-RobotConstants.ROBOT_WIDTH, -RobotConstants.ROBOT_LENGTH);
                wheel = new WPI_TalonSRX(RobotConstants.FR_DRIVE);
                steering = new WPI_TalonSRX(RobotConstants.FR_STEERING);
                zeroPos = STEERING_ENC_ZERO_BR;
                break;
        }
        steering.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, MOTION_MAGIC_IDX, TIMEOUT);
        steering.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT);
        steering.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT);
        steering.selectProfileSlot(SLOT_IDX, MOTION_MAGIC_IDX);
        steering.config_kF(SLOT_IDX, kF, TIMEOUT);
        steering.config_kP(SLOT_IDX, kP, TIMEOUT);
        steering.config_kI(SLOT_IDX, kI, TIMEOUT);
        steering.config_kD(SLOT_IDX, kD, TIMEOUT);
        steering.configMotionCruiseVelocity((int) (MAX_POSSIBLE_VELOCITY / 2.5), TIMEOUT);
        steering.configMotionAcceleration((int) (MAX_POSSIBLE_VELOCITY / 1.5), TIMEOUT);
    }

    public Vector2d getPosVec() {
        return positionVec;
    }

    public int getPosition() {
        return steering.getSelectedSensorPosition(MOTION_MAGIC_IDX);
    }
    
    public int getPositionTrunc() {
        return getPosition() & 0xFFFFD000;
    }

    public double getPositionRad() {
        return (double)getPositionTrunc()/(double)STEERING_COUNTS_PER_REV * Math.PI * 2.0;
    }
    
    public void setPosition(int target) {
        int set = (getPosition() & 0xFFFFD000) + target;
        steering.set(ControlMode.MotionMagic, set);
    }

    public void setPositionRads(double ang) {
        double raw = MathUtil.wrapAngleRad(ang) / (2 * Math.PI) * STEERING_COUNTS_PER_REV + zeroPos;
        setPosition((int)raw);
    }

    public void setDrivePower(double pow) {
        wheel.set(pow);
    }

    public void setToVectorDumb(Vector2d drive) {
        setPositionRads(drive.getAngle());
        setDrivePower(drive.getMagnitude());
    }

    public void setToVectorSmart(Vector2d drive) {
        double pow = drive.getMagnitude();
        if (Math.abs(drive.getAngle() - getPositionRad()) > Math.PI) {
            drive = drive.scale(-1);
            pow *= -1;
        }
        setPositionRads(drive.getAngle());
        setDrivePower(pow);
    }

}
