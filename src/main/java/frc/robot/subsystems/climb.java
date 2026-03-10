package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.idConstants;
import frc.robot.subsystems.automations.AutoAlign;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Climb extends SubsystemBase {
    private static Climb instance = null;

    private SparkFlex climbMotor;
    private SmartMotorControllerConfig vortexConfig;
    private SmartMotorController climbingMotor;

    private AutoAlign align = AutoAlign.getInstance();
    private ElevatorConfig climbConfig;
    private MechanismPositionConfig robotProperties;
    private Elevator climbingSystem;
    
    private LEDS leds = LEDS.getInstance();

    private boolean isUp = false;

    private Climb() {
        climbMotor = new SparkFlex(idConstants.vortex_C1, MotorType.kBrushless);

        vortexConfig = new SmartMotorControllerConfig(this)
                .withClosedLoopController(0.00016541, 0, 0, RPM.of(6784), RotationsPerSecondPerSecond.of(2500))
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(25, 1)))
                .withIdleMode(MotorMode.BRAKE)
                .withTelemetry("ClimbMotor", TelemetryVerbosity.HIGH)
                .withStatorCurrentLimit(Amps.of(40))
                .withMotorInverted(true)
                .withClosedLoopRampRate(Seconds.of(0.25))
                .withOpenLoopRampRate(Seconds.of(0.25))
                .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withMechanismCircumference(Meters.of(0.06)); 

        climbingMotor = new SparkWrapper(climbMotor, DCMotor.getNeoVortex(1), vortexConfig);

        robotProperties = new MechanismPositionConfig()
                .withMaxRobotHeight(Meters.of(0.781175))
                .withMaxRobotLength(Meters.of(0.6858))
                .withRelativePosition(new Translation3d(Meters.of(0.3127375), Meters.of(-0.05785), Meters.of(0.05855431)));

        climbConfig = new ElevatorConfig(climbingMotor)
                .withStartingHeight(Meters.of(0.429))
                .withSoftLimits(Meters.of(0.429), Meters.of(0.536))
                .withHardLimits(Meters.of(0.429), Meters.of(0.6))
                .withTelemetry("Climb", TelemetryVerbosity.HIGH)
                .withMechanismPositionConfig(robotProperties)
                .withMass(Kilograms.of(0.7632));

        climbingSystem = new Elevator(climbConfig);
    }

    public void setArmPos(boolean up) {
        isUp = up;
        if (up) {
            climbingSystem.setHeight(Meter.of(0.53)).schedule();
        } else {
            climbingSystem.setHeight(Meter.of(0.429)).schedule();
        }
    }

    public void toggleClimb() {
        setArmPos(!isUp);
    }

    public void climbInputHandler(int dPad) {
        if (dPad == 180) { // DPAD Down
            toggleClimb();
            leds.climbWave();
        }
    }

    public void sysId(double voltage, double step, double duration) {
        climbingSystem.sysId(Volts.of(voltage), Volts.of(step).per(Second), Seconds.of(duration)).schedule();
    }

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }
}
