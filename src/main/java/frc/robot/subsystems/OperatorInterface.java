package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.automations.Vision;
import frc.robot.test.ShooterTest;
import edu.wpi.first.wpilibj.XboxController;

public class OperatorInterface extends SubsystemBase {
    private static OperatorInterface instance = null;
    private XboxController controller1;
    private Drive drivetrain = Drive.getInstance();
    private Vision vision = Vision.getInstance();
    private Intake ballIntake = Intake.getInstance();
    private Shooter ballShooter = Shooter.getInstance();
    private Climb climber = Climb.getInstance();
    private Indexer indexer = Indexer.getInstance();
    private Telemetry telemetry = Telemetry.getInstance();
    private ShooterTest shooterTest = ShooterTest.getInstance();

    private int lastPOV = -1;

    private OperatorInterface() {
        controller1 = new XboxController(0);
    }

    private void updateDrive() {
        drivetrain.driveInputHandler(
            -controller1.getRawAxis(1), 
            -controller1.getRawAxis(0), 
            -controller1.getRawAxis(4), 
            controller1.getLeftBumperButton()
        );
        drivetrain.updatePoseEstimator();
    }

    private void updateClimb() {
        climber.climbInputHandler(controller1.getPOV());
    }
    private void updateIntake() {
        boolean outtakeButton = controller1.getPOV() == 90; // DPAD Right
        boolean deployButton = controller1.getPOV() == 270; // DPAD Left
        
        ballIntake.intakeInputHandler(
            controller1.getLeftTriggerAxis(),
            outtakeButton,
            deployButton && (controller1.getPOV() != lastPOV), 
            controller1.getRightBumper()
        );
        lastPOV = controller1.getPOV();
    }
    
    private void updateShooter() {
        double rightTrigger = controller1.getRightTriggerAxis();
        ballShooter.shooterInputManager(rightTrigger);
        
        boolean outtakeButton = controller1.getPOV() == 90;
        indexer.indexerInputHandler(ballShooter.isAtTargetSpeed(), outtakeButton);
    }

    private void updateTelemetry() {
        telemetry.update();
    }

    private void updateVision() {
        vision.updateSimVision(drivetrain.getRobotPose());
    }
    
    @Override
    public void periodic() {
        updateDrive();
        updateVision();
        updateTelemetry();
        updateIntake();
        updateShooter();
        updateClimb();
    }

    public static OperatorInterface getInstance() {
        if (instance == null) {
            instance = new OperatorInterface();
        }
        return instance;
    }
}
