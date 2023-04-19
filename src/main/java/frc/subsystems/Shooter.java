/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;


import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

    private CANSparkMax shooterTop;
    private CANSparkMax shooterBottom;
    private double kP = 0.000100; //0.000400;
    private double kI = 0.000001;
    private int kD = 0;
    private SparkMaxPIDController topPid;
    private SparkMaxPIDController bottomPid;
    public static double topSpinFactor = 0.6;

    public Shooter() {
        shooterTop = new CANSparkMax(RobotMap.SHOOTER_TOP, CANSparkMaxLowLevel.MotorType.kBrushless);
        shooterTop.restoreFactoryDefaults();
        shooterBottom = new CANSparkMax(RobotMap.SHOOTER_BOTTOM, CANSparkMaxLowLevel.MotorType.kBrushless);
        shooterBottom.restoreFactoryDefaults();

        topPid = shooterTop.getPIDController();
        topPid.setP(kP);
        topPid.setI(kI);
        topPid.setD(kD);

        bottomPid = shooterBottom.getPIDController();
        bottomPid.setP(kP);
        bottomPid.setI(kI);
        bottomPid.setD(kD);
    }

    public void setShooterVoltage(double percent) {
        shooterTop.set(percent);
        shooterBottom.set(percent);
    }

    public void stopMotors() {
        shooterTop.set(0);
        shooterBottom.set(0);
    }

    public SparkMaxPIDController getShooterTopPIDController() {
        return topPid;
    }

    public SparkMaxPIDController getShooterBottomPIDController() {
        return bottomPid;
    }

    public double getShooterTopRPM() {
        return shooterTop.getEncoder().getVelocity();
    }

    public double getShooterBottomRPM() {
        return shooterBottom.getEncoder().getVelocity();
    }

    public double shootDistance(double distance){
        double shooterSpeed = distance/10; //need testing for exact equation

        return shooterSpeed;
    }

    public void setPIDWithTopSpin(double topShooterTargetRPM){
        double bottomTargetRPM = topShooterTargetRPM * topSpinFactor;
        topPid.setReference(topShooterTargetRPM, ControlType.kVelocity);
        bottomPid.setReference(bottomTargetRPM, ControlType.kVelocity);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Top RPM", shooterTop.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Bottom RPM", shooterBottom.getEncoder().getVelocity());

    }
}