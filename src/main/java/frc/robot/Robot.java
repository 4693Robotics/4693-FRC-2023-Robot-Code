// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Librares
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
//import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Robot extends TimedRobot {

// Random probably useless thing & Pneumatics
  private static final int kJoystickChannel = 0;
  DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

// Drive Type and Controller Definitions
  private DifferentialDrive m_robotDrive;
  private XboxController m_stick = new XboxController(0);
  private XboxController c_stick = new XboxController(1);

//drive sense
  private static final double RotateSensitivity = 0.9;
  private static final double DriveSensitivity = 0.5;
  private static final double RotateSenseSlowMultiplyer = 0.8;
  private static final double DriveSenseSlowMultiplyer = 0.6;
// Arm Motor Controller Definitions
  VictorSPX arm1 = new VictorSPX(5);
  VictorSPX arm2 = new VictorSPX(6);
  
// Gyro Stuff
  AHRS gyro = new AHRS(SerialPort.Port.kUSB1);

  PIDController pid = new PIDController(0.042, 0, .001);


  @Override
  public void robotInit() {

  // Drive Train Motor Controller Definitions
    CANSparkMax frontLeft = new CANSparkMax(1,MotorType.kBrushed);
    CANSparkMax rearLeft = new CANSparkMax(2,MotorType.kBrushed);
    CANSparkMax frontRight = new CANSparkMax(3,MotorType.kBrushed);
    CANSparkMax rearRight = new CANSparkMax(4,MotorType.kBrushed);

    MotorControllerGroup m_left = new MotorControllerGroup(frontLeft, rearLeft);
    MotorControllerGroup m_right = new MotorControllerGroup(frontRight, rearRight);

  // Invert the right side motors.
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);

  // Drive Train Composition
    m_robotDrive = new DifferentialDrive(m_left , m_right);
    
  // Drive Controller Re-definition (Might be unneeded IDK)
    m_stick = new XboxController(kJoystickChannel);

  // Camera
    CameraServer.startAutomaticCapture();

  // Smart Dashboard Variables   
    SmartDashboard.putNumber("porportional", .042);
    SmartDashboard.putNumber("integral", 0);
    SmartDashboard.putNumber("Derivitive", .001);

  //Gyro Calibration
    gyro.calibrate();
  }
  

	@Override
	public void autonomousInit() {

  //Safety
    m_robotDrive.setSafetyEnabled(false);

  //Proportianality 
    pid = new PIDController(SmartDashboard.getNumber("porportional", .042),SmartDashboard.getNumber("integral", 0),SmartDashboard.getNumber("Derivitive", .001));

  //Prep
    arm2.set(VictorSPXControlMode.PercentOutput,-.6);
    Timer.delay(.5);
    arm2.set(VictorSPXControlMode.PercentOutput,-.18);
    arm1.set(VictorSPXControlMode.PercentOutput,.65);
    m_robotDrive.arcadeDrive(.85, 0);    // drive forwards quarter speed
    Timer.delay(1.5);                             // for 2 seconds
    m_robotDrive.arcadeDrive(0, 0); 
	}


	@Override
	public void autonomousPeriodic() {

  // Gyro Stuff
    m_robotDrive.arcadeDrive(pid.calculate(gyro.getRoll(),0),0);
  }


	@Override
	public void teleopInit() {

		m_robotDrive.setSafetyEnabled(true);
	}


  @Override
  public void teleopPeriodic() {
  // Drive Controls
    if (m_stick.getRawButton(6)) {
      m_robotDrive.arcadeDrive(m_stick.getLeftY()*DriveSenseSlowMultiplyer*DriveSensitivity, -m_stick.getRightX()*RotateSenseSlowMultiplyer*RotateSensitivity);
    }else {
      m_robotDrive.arcadeDrive(m_stick.getLeftY()*DriveSensitivity, -m_stick.getRightX()*RotateSensitivity);
    }
  
  //Arm 1 Controls
    double arm1move = c_stick.getRightY()* .65;    
    arm1.set(VictorSPXControlMode.PercentOutput,arm1move);

  //Arm 2 Controls
    if (c_stick.getRawButton(2)) {
      double arm2move = c_stick.getLeftY() * .4 -.2; 
      arm2.set(VictorSPXControlMode.PercentOutput,arm2move);
    }else {
      double arm2move = c_stick.getLeftY() * .3 -.2;              //Multiplyer for stick value so we don't kill the arm  
      arm2.set(VictorSPXControlMode.PercentOutput,arm2move);  //Movement for Arm 2
    }

  //Pneumatic Controls
    if (c_stick.getRawButton(3)) {
      piston.set(Value.kForward);
    }

    if (c_stick.getRawButton(1)) {
      piston.set(Value.kReverse);
    }
  }
}