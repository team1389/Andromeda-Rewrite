package frc.commands;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.subsystems.Shooter;
import frc.utils.SizeLimitedQueue;

import javax.naming.ldap.Control;

public class ShootWithSensors extends SequentialCommandGroup {

    private double shooterTargetRPM;
    private double bottomTargetRPM;
    private double conveyorPercent;
    private SparkMaxPIDController topPID, bottomPID;
    double kV = 473;
    boolean stopShooterRunning;
    String distance;

    public ShootWithSensors() {
        addRequirements(Robot.shooter, Robot.conveyor, Robot.indexer);

        double distanceToTarget = SmartDashboard.getNumber("Distance To Target", 0);

        if(distanceToTarget <= 200) {
            shooterTargetRPM = 4600;
            distance = "Short";
        }
        else if(distanceToTarget > 200 && distanceToTarget <= 420) {
            shooterTargetRPM = 5050;
            distance = "Medium";
        }
        else if(distanceToTarget > 420) {
            shooterTargetRPM = 5400;
            distance = "Long";
        }

        SmartDashboard.putString("Distance", distance);

        topPID = Robot.shooter.getShooterTopPIDController();
        bottomPID = Robot.shooter.getShooterBottomPIDController();
        bottomTargetRPM = shooterTargetRPM * Shooter.topSpinFactor;

        conveyorPercent = 0.7;
        addCommands(
            new InstantCommand(() -> topPID.setReference(shooterTargetRPM/kV, CANSparkMax.ControlType.kVoltage)),
            new InstantCommand(()-> bottomPID.setReference(bottomTargetRPM/kV, CANSparkMax.ControlType.kVoltage)),
            new ParallelCommandGroup(new WaitCommand(1),
                    new AdjustToTarget()),
                    new InstantCommand(() -> Robot.indexer.runIndexer(1)),
                    new InstantCommand(() -> Robot.conveyor.runConveyor(conveyorPercent)),
                    new WaitCommand(10),
                    new InstantCommand(()->Robot.shooter.stopMotors()),
                    new InstantCommand(()->Robot.indexer.stopIndexer()),
                    new InstantCommand(()->Robot.conveyor.stopConveyor()),
                    new InstantCommand(()-> topPID.setReference(0, CANSparkMax.ControlType.kVelocity)),
                    new InstantCommand(()->bottomPID.setReference(0, CANSparkMax.ControlType.kVelocity)),
                    new InstantCommand(()->NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1))
        );
    }

    //TODO: Switch this to use the method from shooter after its been tested

    

    public static class WaitUntilAtSpeed extends CommandBase {

        private double shooterTargetRPM, bottomTargetRPM;
        private double tolerance;
        private SizeLimitedQueue recentTopErrors = new SizeLimitedQueue(15);
        private SizeLimitedQueue recentBottomErrors = new SizeLimitedQueue(15);
        private Timer timer = new Timer();

        public WaitUntilAtSpeed(double shooterTargetRPM, double bottomTargetRPM) {
            addRequirements(Robot.shooter);
            this.shooterTargetRPM = shooterTargetRPM;
            this.bottomTargetRPM = bottomTargetRPM;
            tolerance = 1.5;
        }


        @Override
        public void execute() {
            SmartDashboard.putBoolean("pid-ing", true);

            double topError = shooterTargetRPM - Robot.shooter.getShooterTopRPM();
            recentTopErrors.addElement(topError);
            SmartDashboard.putNumber("average top error", recentTopErrors.getAverage());

            double bottomError = bottomTargetRPM - Robot.shooter.getShooterBottomRPM();
            recentBottomErrors.addElement(bottomError);
            SmartDashboard.putNumber("average bottom error", recentBottomErrors.getAverage());
        }

        @Override
        public void end(boolean interrupted) {
            SmartDashboard.putBoolean("pid-ing", false);
        }

        @Override
        public boolean isFinished() {
            return tolerance >= Math.abs(recentTopErrors.getAverage()) &&
                    tolerance >= Math.abs(recentBottomErrors.getAverage());
        }
    }



}