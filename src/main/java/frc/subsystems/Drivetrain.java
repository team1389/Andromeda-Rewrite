package frc.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
    public AHRS ahrs = new AHRS(SerialPort.Port.kMXP);
    public RelativeEncoder leftLeaderEncoder, rightLeaderEncoder;
    private DifferentialDrive differentialDrive;
    private CANSparkMax rightLeader, rightFollower, leftLeader, leftFollower;
    private SparkMaxPIDController leftPid, rightPid;

    public Drivetrain() {
        rightLeader = new CANSparkMax(RobotMap.RIGHT_DRIVE_LEADER, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightFollower = new CANSparkMax(RobotMap.RIGHT_DRIVE_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightLeader.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();
        rightFollower.follow(rightLeader);


        leftLeader = new CANSparkMax(RobotMap.LEFT_DRIVE_LEADER, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftFollower = new CANSparkMax(RobotMap.LEFT_DRIVE_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        leftFollower.follow(leftLeader);

        differentialDrive = new DifferentialDrive(leftLeader, rightLeader);

        leftLeaderEncoder = leftLeader.getEncoder();
        leftLeaderEncoder.setPositionConversionFactor(-1);
        rightLeaderEncoder = rightLeader.getEncoder();

        leftLeaderEncoder.setPosition(0);
        rightLeaderEncoder.setPosition(0);
        ahrs.reset();

        leftPid = leftLeader.getPIDController();
        rightPid = rightLeader.getPIDController();

        leftPid.setP(0.1);
        leftPid.setI(0);
        leftPid.setD(0);

        rightPid.setP(0.1);
        rightPid.setI(0);
        rightPid.setD(0);
        rightLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);

    }

    public double getLeftPosition() {
        return leftLeaderEncoder.getPosition();
    }

    public double getRightPosition() {
        return rightLeaderEncoder.getPosition();
    }

    public SparkMaxPIDController getLeftPidController() {
        return leftPid;
    }

    public SparkMaxPIDController getRightPidController() {
        return rightPid;
    }

    public double rightLeaderEncoder() {
        return rightLeaderEncoder.getPosition();
    }

    public void drive(double leftY, double rightX, boolean isQuickTurn, boolean decreaseSpeed) {
        if (decreaseSpeed) {
            differentialDrive.curvatureDrive(leftY / 2, rightX / 2, isQuickTurn);
        } else {
            differentialDrive.curvatureDrive(leftY, rightX, isQuickTurn);
        }
    }

    public void set(double leftPower, double rightPower) {
        leftLeader.set(leftPower);
        rightLeader.set(-rightPower);
    }

    public double getAngle() {
        return ahrs.getAngle();
    }

    public double encoderCountsPerRevolution() {
        return leftLeaderEncoder.getCountsPerRevolution();
    }

    public void resetEncoders() {
        leftLeaderEncoder.setPosition(0);
        rightLeaderEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
    }

    public void setCoast() {
        leftLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setBrake() {
        leftLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

}
