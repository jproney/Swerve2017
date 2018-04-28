package subsystems;

import com.ctre.CANTalon;

import mathUtil.MathUtil;
import mathUtil.Vector2d;
import robot.RobotConstants;

public class SwerveModule {

    public enum ModuleID {
        FR, FL, BL, BR;
    }

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.0;
    public static final double cruise = 0.0;
    public static final double accel = 0.0;

    public static final int STEERING_ENC_SCALE = 4096 * 2;
    public static final double STEERING_ENC_ZERO_FL = 0.0;
    public static final double STEERING_ENC_ZERO_FR = 0.0;
    public static final double STEERING_ENC_ZERO_BL = 0.0;
    public static final double STEERING_ENC_ZERO_BR = 0.0;

    public double steeringEncZero;
    private Vector2d positionVec;
    private CANTalon wheel;
    private CANTalon steering;
    private ModuleID modID;
    private final double zeroPos;

    public SwerveModule(ModuleID id) {
        modID = id;
        switch (modID) {
            case FR:
                positionVec = new Vector2d(RobotConstants.ROBOT_WIDTH, RobotConstants.ROBOT_LENGTH);
                wheel = new CANTalon(RobotConstants.FR_DRIVE);
                steering = new CANTalon(RobotConstants.FR_STEERING);
                zeroPos = STEERING_ENC_ZERO_FL;
                break;
            case FL:
                positionVec = new Vector2d(-RobotConstants.ROBOT_WIDTH, RobotConstants.ROBOT_LENGTH);
                wheel = new CANTalon(RobotConstants.FL_DRIVE);
                steering = new CANTalon(RobotConstants.FL_STEERING);
                zeroPos = STEERING_ENC_ZERO_FR;
                break;
            case BR:
                positionVec = new Vector2d(RobotConstants.ROBOT_WIDTH, -RobotConstants.ROBOT_LENGTH);
                wheel = new CANTalon(RobotConstants.FR_DRIVE);
                steering = new CANTalon(RobotConstants.FR_STEERING);
                zeroPos = STEERING_ENC_ZERO_BL;
                break;
            case BL:
            default:
                positionVec = new Vector2d(-RobotConstants.ROBOT_WIDTH, -RobotConstants.ROBOT_LENGTH);
                wheel = new CANTalon(RobotConstants.FR_DRIVE);
                steering = new CANTalon(RobotConstants.FR_STEERING);
                zeroPos = STEERING_ENC_ZERO_BR;
                break;
        }
        steering.changeControlMode(CANTalon.TalonControlMode.Position);
        steering.setP(kP);
        steering.setI(kI);
        steering.setD(kD);
        steering.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Absolute);
        steering.configEncoderCodesPerRev(STEERING_ENC_SCALE);

        wheel.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    }

    public Vector2d getPosVec() {
        return positionVec;
    }

    public double getPositionRaw() {
        return steering.getPulseWidthPosition();
    }

    public double getPositionRevs() {
        steering.setEncPosition(steering.getPulseWidthPosition());
        return steering.getPosition();
    }

    public double getPositionRad() {
        return MathUtil.wrapAngleRad(getPositionRevs() * Math.PI * 2);
    }

    public void setPositionCounts(double pos) {
        steering.set(pos);
    }

    public void setPositionRads(double ang) {
        double raw = MathUtil.wrapAngleRad(ang) / (2 * Math.PI) * STEERING_ENC_SCALE + zeroPos;
        steering.set(raw);
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
        if (Math.abs(drive.getMagnitude() - getPositionRad()) > Math.PI) {
            drive = drive.scale(-1);
            pow *= -1;
        }
        setPositionRads(drive.getAngle());
        setDrivePower(pow);
    }

}
