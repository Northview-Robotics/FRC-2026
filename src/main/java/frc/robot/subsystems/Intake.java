package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.idConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    private TalonFX intakeMotor;
    private TalonFX hopperMotor;
    private SparkMax pivotMotor;
    private Telemetry telemetry;

    private final SmartMotorControllerConfig falconConfig;
    private final SmartMotorControllerConfig neoConfig;
    private SmartMotorController pivotingSystem;
    private SmartMotorController hopperSystem;
    private SmartMotorController intakingSystem;

    private LEDS leds = LEDS.getInstance();

    private Arm pivot;
    private ArmConfig pivotConfig;

    private boolean hopperDeployed = false;
    private boolean intakeActive = false;

    private Intake() {
        hopperMotor = new TalonFX(idConstants.falcon500_I1);
        intakeMotor = new TalonFX(idConstants.falcon500_I2);
        pivotMotor = new SparkMax(idConstants.neo_I3, MotorType.kBrushless);

        falconConfig = new SmartMotorControllerConfig(this)
                .withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                .withIdleMode(MotorMode.COAST)
                .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
                .withStatorCurrentLimit(Amps.of(20))
                .withMotorInverted(true)
                .withClosedLoopRampRate(Seconds.of(0.25))
                .withOpenLoopRampRate(Seconds.of(0.25))
                .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                .withControlMode(ControlMode.CLOSED_LOOP);

        neoConfig = new SmartMotorControllerConfig(this)
                .withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                .withIdleMode(MotorMode.COAST)
                .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
                .withStatorCurrentLimit(Amps.of(20))
                .withMotorInverted(true)
                .withClosedLoopRampRate(Seconds.of(0.25))
                .withOpenLoopRampRate(Seconds.of(0.25))
                .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                .withControlMode(ControlMode.CLOSED_LOOP);

        pivotingSystem = new SparkWrapper(pivotMotor, DCMotor.getNEO(1), neoConfig);

        pivotConfig = new ArmConfig(pivotingSystem)
                .withLength(Meters.of(0.3366))
                .withSoftLimits(Degrees.of(26.5), Degrees.of(215))
                .withHardLimit(Degrees.of(26.5), Degrees.of(215))
                .withTelemetry("Pivot", TelemetryVerbosity.HIGH)
                .withMass(Kilograms.of(2.714))
                .withStartingPosition(Degrees.of(0));

        pivot = new Arm(pivotConfig);

        hopperSystem = new TalonFXWrapper(hopperMotor, DCMotor.getFalcon500(1), falconConfig);
        intakingSystem = new TalonFXWrapper(intakeMotor, DCMotor.getFalcon500(1), falconConfig);
    }

    public double calcIntakingVolts() {
        telemetry = Telemetry.getInstance();
        ChassisSpeeds chassisVel = telemetry.currentVelocity;
        if (chassisVel == null) return 7.0;
        double botVelocity = Math.hypot(chassisVel.vxMetersPerSecond, chassisVel.vyMetersPerSecond);
        double maxVelocity = Constants.maxDriveSpeed;
        double maxVolts = Constants.maxVolts;

        double scaling = 0.7 - (botVelocity / maxVelocity);
        return maxVolts * scaling;
    }

    public void runIntake(double voltage) {
        intakingSystem.setVoltage(Volts.of(voltage));
    }

    public void runHopper(double voltage) {
        hopperSystem.setVoltage(Volts.of(voltage));
    }

    public void setAngle(Angle angle) {
        pivot.setAngle(angle);
    }

    public void setIntakePivot() {
        setAngle(Degrees.of(Constants.intakeAngle));
    }

    public void setShootingPivot() {
        setAngle(Degrees.of(Constants.shootingAngle));
    }

    public void setStowedPivot() {
        setAngle(Degrees.of(Constants.rockingAngles[0]));
    }

    public void setIntakeActive(boolean active) {
        this.intakeActive = active;
    }

    public void toggleHopperDeployment() {
        hopperDeployed = !hopperDeployed;
        if (hopperDeployed) {
            setIntakePivot();
        } else {
            setStowedPivot();
        }
    }

    public void oscillateIntake() {
        double k = (2 * Math.PI) / Constants.osilationTIme;
        double magnitude = 5; 
        double offset = Math.sin(k * Timer.getFPGATimestamp()) * magnitude;
        setAngle(Degrees.of(Constants.intakeAngle + offset));
    }

    public void intakeInputHandler(double intakeTrigger, boolean outtakeButton, boolean deployButton, boolean oscillateButton) {
        if (deployButton) {
            toggleHopperDeployment();
        }

        if (outtakeButton) {
            runIntake(-6);
            runHopper(-6);
            leds.intakeSolid();
        } else if (intakeTrigger > 0.3 || intakeActive) {
            runIntake(calcIntakingVolts());
            runHopper(4);
            leds.intakeSolid();
            if (oscillateButton) {
                oscillateIntake();
            } else if (hopperDeployed || intakeActive) {
                setIntakePivot();
            }
        } else {
            runIntake(0);
            runHopper(0);
            if (oscillateButton) {
                oscillateIntake();
            }
        }
    }


    public void sysId(double voltage, double step, double duration) {
        pivot.sysId(Volts.of(voltage), Volts.of(step).per(Second), Seconds.of(duration)).schedule();
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }
}
