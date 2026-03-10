// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RobotContainer;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.automations.Vision;
import frc.robot.subsystems.automations.AutoAlign;

public class Robot extends TimedRobot {
  public Drive drivetrain;
  public OperatorInterface oi;
  public Shooter ballShooter;
  public Intake ballIntake;
  public Climb climber;
  public Indexer indexer;
  public AutoAlign align;
  public Vision vision;
  public RobotContainer rc;
  public Command getAutonomousCommand;
  private LEDS leds = LEDS.getInstance();


  public Robot() {
    drivetrain = Drive.getInstance();
    vision = Vision.getInstance();
    ballIntake = Intake.getInstance();
    align = AutoAlign.getInstance();
    ballShooter = Shooter.getInstance();
    climber = Climb.getInstance();
    indexer = Indexer.getInstance();
    oi = OperatorInterface.getInstance();
    rc = new RobotContainer();
  }

  @Override
  public void robotInit() {
    DataLogManager.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    getAutonomousCommand = rc.getAutonomousCommand();
    
    if (getAutonomousCommand != null){
      getAutonomousCommand.schedule();
   }
  }

  @Override
  public void autonomousPeriodic() {
    leds.autonWave();
  }

  @Override
  public void teleopInit() {
    if (getAutonomousCommand != null){
      getAutonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    leds.teleopBlink();
  }
}
