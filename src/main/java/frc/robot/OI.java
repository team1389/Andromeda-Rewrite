package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.commands.*;

public class OI {
    public XboxController driveController, manipController;
    DriveWithCurvature driveWithCurvature = new DriveWithCurvature();

    private JoystickButton bBtn, aBtn, lBumper, yBtn, xBtn, rBumper, lTrigger;
    private JoystickButton driveYBtn;

    public OI() {
        driveController = new XboxController(0);
        manipController = new XboxController(1);

        //Intake Buttons
        aBtn = new JoystickButton(manipController, XboxController.Button.kA.value);
        aBtn.toggleOnTrue(new RunIntakeAndSendBallToIndexer());

        bBtn = new JoystickButton(manipController, XboxController.Button.kB.value);
        bBtn.toggleOnTrue(new Outtake());

        //Climber Commands
        rBumper = new JoystickButton(manipController, XboxController.Button.kRightBumper.value);
        rBumper.whileTrue(new WinchClimber());

        xBtn = new JoystickButton(manipController, XboxController.Button.kX.value);
        xBtn.onTrue(new ExtendClimber());


        yBtn = new JoystickButton(manipController, XboxController.Button.kY.value);
        yBtn.onTrue(new RetractClimber());



        //Shooter Commands
        lBumper = new JoystickButton(manipController, XboxController.Button.kLeftBumper.value);
        //3300-3500rpm is control panel shot
        lBumper.whileTrue(new DistanceShoot()); //4400
        //lBumper.whileTrue(new AdjustToTarget());
        //lBumper.whileTrue(new TestShooterSpeed());


        driveYBtn = new JoystickButton(driveController, XboxController.Button.kY.value);
        driveYBtn.whileTrue(new InstantCommand(() -> NetworkTableInstance.getDefault().getTable("limelight")
                .getEntry("ledMode").setNumber(3)));


        Robot.drivetrain.setDefaultCommand(driveWithCurvature);
        //Robot.conveyor.setDefaultCommand(new DescendConveyor());

    }


}