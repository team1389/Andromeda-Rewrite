package frc.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GetDistanceToTarget extends CommandBase {
    private double TARGET_HEIGHT_INCHES = 83.25; //83.25 on real field, 95 on shelf
    private double CAMERA_ANGLE_DEGREES = 27.21212218; //REMEMBER TO CHANGE THIS EVERY MATCH IF WE ADJUST THE SHOOTER
    private double CAMERA_HEIGHT_INCHES = 26;

    private double ty;
    private double distanceToTarget = 0;

    private Timer timer = new Timer();

    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(3);
        distanceToTarget = (TARGET_HEIGHT_INCHES-CAMERA_HEIGHT_INCHES)/(Math.tan(Math.toRadians(CAMERA_ANGLE_DEGREES+ty)));

        SmartDashboard.putNumber("Distance To Target", distanceToTarget);
    }

    @Override
    public void end(boolean interrupted) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    @Override
    public boolean isFinished() {
        return distanceToTarget != 0 || timer.get() > 1;
    }

}
