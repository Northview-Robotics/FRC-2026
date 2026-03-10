package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
  //Global
  
  //Math/Calculations 
  public static final double secondsPerMinute = 60.0;
  public static final double driveCircumferenceMeters = 0.2393893602;
  //YAGSL
  public static final double maxDriveSpeed = Units.feetToMeters(16);
  public static final double maxVolts = 12;
  public static final Pose2d startPose = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
  public static final double deadband = 0.05;

  //Vision 
  public static final double visionX = 0.2;
  public static final double visionY = 0.0;
  public static final double visionRotation = 0.0; 

  //Intake Subsystems
  public static final int[] rockingAngles = {0,90};
  public static final int osilationTIme = 3;
  public static final int shootingAngle = 15;
  public static final int intakeAngle = 215;
  public static final double intakeSysIdMaxVoltage = 10;
  public static final double intakeSysIdStep = 1.0;
  public static final double intakeSysIdDuration = 5.0;

  //Climb Subsystem
  public static final double climbSysIdMaxVoltage = 10;
  public static final double climbSysIdStep = 1.0;
  public static final double climbSysIdDuration = 5.0;
  
  //Flywheel Subsystem
  public static final double flyWheelSysIdMaxVoltage = 10;
  public static final double flyWheelSysIdStep = 1;
  public static final double flyWheelSysIdDuration = 5;

  //Hood Subsystem
  public static final double hoodSysIdMaxVoltage = 10;
  public static final double hoodSysIdStep = 1;
  public static final double hoodSysIdDuration = 5;
  public static final double startingHoodAngle = 20; // Why was this 5????
  public static final Distance hoodArmLength = Meters.of(0.33176);
  public static final Mass hoodMass = Kilograms.of(1.451496);

  public static final Mass robotMass = Kilograms.of(48);

}