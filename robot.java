// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Robot extends TimedRobot {
  // declaring stuff
  private final DifferentialDrive m_robotDrive;
  private final XboxController m_controller;
  private final SparkMax m_leftMotor;
  private final SparkMax m_rightMotor;
  private final SparkMax m_leftFollowerMotor;
  private final SparkMax m_rightFollowerMotor;
  private final SparkMax m_launcherMotor;
  private Timer timer;

  public final class Constants {
    public static final class DriveConstants {
        public static final int KLEFT_LEADER_ID = 1;
        public static final int KLEFT_FOLLOWER_ID = 2;
        public static final int KRIGHT_LEADER_ID = 3;
        public static final int KRIGHT_FOLLOWER_ID = 4;
    
        public static final int KDRIVE_MOTOR_CURRENT_LIMIT = 60;
      }
    
      public static final class CoralReleaseConstants {
        public static final int KROLLER_MOTOR_ID = 5;
        public static final int KROLLER_MOTOR_CURRENT_LIMIT = 60;
        public static final double KROLLER_MOTOR_VOLTAGE_COMP = 10;
        public static final double KROLLER_EJECT_VALUE = 0.44;
      }
    
      public static final class OperatorConstants {
        public static final int KDRIVER_CONTROLLER_PORT = 0;
        public static final int KOPERATOR_CONTROLLER_PORT = 1;
      }
  
      public static final class AutonomousConstants {
        public static final String KDEFAULT_AUTONOMOUS = "Default";
        public static final String KCUSTOM_AUTONMOUS = "My Auton";
      }

  }
  
  /** Called once at the beginning of the robot program. */
  public Robot() {
    // creating timer
    this.timer = new Timer();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    this.m_leftMotor = new SparkMax(4, MotorType.kBrushed);
    this.m_rightMotor = new SparkMax(3, MotorType.kBrushed);
    this.m_leftFollowerMotor = new SparkMax(2, MotorType.kBrushed);
    this.m_rightFollowerMotor = new SparkMax(1, MotorType.kBrushed);
    this.m_launcherMotor = new SparkMax(5, MotorType.kBrushed);
    SparkBaseConfig driveconfig = new SparkMaxConfig();
    driveconfig.idleMode(IdleMode.kBrake);
    driveconfig.smartCurrentLimit(80);

    driveconfig.inverted(true);
    this.m_launcherMotor.configure(driveconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    SparkBaseConfig leftMotorConfig = new SparkMaxConfig();
    SparkBaseConfig rightMotorConfig = new SparkMaxConfig();
    SparkBaseConfig leftFollowerConfig = new SparkMaxConfig();
    SparkBaseConfig rightFollowerConfig = new SparkMaxConfig();

    // leftMotorConfig.inverted(false);?
    leftMotorConfig.idleMode(IdleMode.kBrake);
    leftMotorConfig.smartCurrentLimit(80);

    rightMotorConfig.inverted(true);
    rightMotorConfig.idleMode(IdleMode.kBrake);
    rightMotorConfig.smartCurrentLimit(80);

    leftFollowerConfig.follow(4);
    rightFollowerConfig.follow(3);

    this.m_rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.m_leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.m_leftFollowerMotor.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.m_rightFollowerMotor.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.m_robotDrive = new DifferentialDrive(this.m_leftMotor::set, this.m_rightMotor::set);
    this.m_controller = new XboxController(0);

    SendableRegistry.addChild(this.m_robotDrive, this.m_leftMotor);
    SendableRegistry.addChild(this.m_robotDrive, this.m_rightMotor);
    SendableRegistry.addChild(this.m_robotDrive, this.m_launcherMotor);
  }
 
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive((0.85)*m_controller.getLeftY(), (0.85)*m_controller.getRightX());
    // this.m_robotDrive.tankDrive(this.m_controller.getLeftY(), this.m_controller.getRightY());
    // seeing if robot should be turning or going straight
    // if (this.m_controller.getRightX() > 0) {
    //    this.m_robotDrive.tankDrive((this.m_controller.getLeftY()-(this.m_controller.getRightX()*1.5)), (this.m_controller.getLeftY()+(this.m_controller.getRightX()*1.5)));
    //  }
    //  else if (this.m_controller.getRightX() < 0) {
    //   this.m_robotDrive.tankDrive((this.m_controller.getLeftY()-(this.m_controller.getRightX()*1.5)), (this.m_controller.getLeftY()+(this.m_controller.getRightX()*1.5)));
    // }
    //  else {
    //   this.m_robotDrive.tankDrive(this.m_controller.getLeftY(), this.m_controller.getLeftY());
    // }

    // triggering launcher
    if (this.m_controller.getAButtonPressed()) {
      this.m_launcherMotor.set(-1);
    }
    if (this.m_controller.getAButtonReleased()){
      this.m_launcherMotor.set(0);
    }
    
    // Shooting coral up so it realligns
    if (this.m_controller.getYButtonPressed()) {
      this.m_launcherMotor.set(1);
    }
    if (this.m_controller.getYButtonReleased()){
      this.m_launcherMotor.set(0);
    }
  }
  @Override
  public void autonomousInit() {
    this.timer.reset();
    this.timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    this.m_robotDrive.tankDrive(0.5, 0.5);
    if (this.timer.hasElapsed(10)) { // the time elapsed is just approx. we gotta test it to see how long
      this.m_robotDrive.tankDrive(0, 0);
      }
    if (this.timer.hasElapsed(10.1)) {
      this.m_launcherMotor.set(-.5);
      }
    if (this.timer.hasElapsed(10.5)) {
      this.m_launcherMotor.set(0);
      }
    }
  } 
