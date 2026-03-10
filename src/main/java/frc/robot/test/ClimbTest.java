package frc.robot.test;
import frc.robot.subsystems.Climb;
import frc.robot.constants.Constants;

public class ClimbTest {
    private static ClimbTest instance = null;
    private Climb climber = Climb.getInstance();

    public void runClimbSysID(){
        climber.sysId(Constants.climbSysIdMaxVoltage, Constants.climbSysIdStep, Constants.climbSysIdDuration);
    }

    public static ClimbTest getInstance(){
        if (instance == null){
            instance = new ClimbTest();
        }
        return instance;
    }
}
