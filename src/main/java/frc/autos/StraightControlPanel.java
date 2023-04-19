package frc.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.commands.*;
import frc.robot.Robot;

/**
 * Shoots and picks up from the control panel and then shoots again
 */
public class StraightControlPanel extends SequentialCommandGroup {

    public ParallelCommandGroup driveAndSendBallToIndexer;
    //If this doesn't get it fast enough, speed up the last shoot command
    public StraightControlPanel() {
        addRequirements(Robot.drivetrain, Robot.shooter);
        double conveyorSpeed = 0.6;
        //slowing conveyor speed to stack the balls in the conveyor
        driveAndSendBallToIndexer = new ParallelCommandGroup(new DriveDistance(145),
                new SendBallToIndexer(conveyorSpeed));

        //Shot can take less time

        addCommands(new ShootWithoutPID(0.5, 1, 1),
                new TurnToAngle(3, false),
                new DriveDistance(10),
                new TurnToAngle(3, false),
                new WaitCommand(0.5),
                new InstantCommand(() -> Robot.intake.runIntake(0.85)),
                driveAndSendBallToIndexer,
                new InstantCommand(() -> Robot.intake.stopIntaking()),
                new DistanceShoot(),
                new InstantCommand(() -> Robot.intake.stopIntaking()),
                new InstantCommand(() -> Robot.shooter.stopMotors())
                );
    }

}
