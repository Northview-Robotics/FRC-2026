package frc.robot.test;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter;
import frc.robot.constants.Constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterTest extends SubsystemBase {
    private static ShooterTest instance = null;
    private Shooter ballShooter = Shooter.getInstance();
    
    private double testHoodInput = 0;
    private double testFlyWheelRPM = 0;

    private ShooterTest() {
        SmartDashboard.putNumber("TestHoodAngle", 0.0);
        SmartDashboard.putNumber("TestRPM", 0.0);
    }

    @Override
    public void periodic() {
        testHoodInput = SmartDashboard.getNumber("TestHoodAngle", 0.0);
        testFlyWheelRPM = SmartDashboard.getNumber("TestRPM", 0.0);
    }
    

    public void SetTestVelocity(boolean SetTestVel){
        if (SetTestVel) {
            ballShooter.setFlyWheelVel(RPM.of(testFlyWheelRPM));
        }
    }

    public void setTestHoodAngle(boolean SetTestHoodAngle){
        if(SetTestHoodAngle) {
            ballShooter.setHoodAngle(Degrees.of(testHoodInput));
        }
    }

    public void runFlyWheelSysID(boolean run){
        if(run) ballShooter.flyWheelSysId(Constants.flyWheelSysIdMaxVoltage, Constants.flyWheelSysIdStep, Constants.flyWheelSysIdDuration);
    }

    public void runHoodSysID(boolean run){
        if(run) ballShooter.hoodSysId(Constants.hoodSysIdMaxVoltage, Constants.hoodSysIdStep, Constants.hoodSysIdDuration);
    }

    public static ShooterTest getInstance(){
        if (instance == null){
            instance = new ShooterTest();
        }
        return instance;
    }
}
