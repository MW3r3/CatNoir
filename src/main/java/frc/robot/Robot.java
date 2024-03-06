package frc.robot;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Timer;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private XboxController m_ControllerMain;
  
  

  // Trigger variables
  private static final int m_leftStickYID = 1;
  private static final int m_rightStickXID = 4;
  private static final int m_leftTriggerID = 2;
  private static final int m_rightTriggerID = 3;


  // Motor power variables
  private double throttle;
  private double difference;
  private double leftMotors;
  private double rightMotors;

  private double rightTriggerMain;
  private double leftTriggerMain;
  private Timer timer = new Timer();
 


  // The CAN ID of the motor connected to the Spark Max
  private static final int leadleftDeviceID = 10;
  private static final int followleftDeviceID = 11;
  private static final int leadrightDeviceID = 12;
  private static final int followrightDeviceID = 13;
 

  private CANSparkMax m_leadleftMotor;
  private CANSparkMax m_followleftMotor;
  private CANSparkMax m_leadrightMotor;
  private CANSparkMax m_followrightMotor;




  @Override
  public void robotInit() {

    // Initialize the motors
    m_leadleftMotor = new CANSparkMax(leadleftDeviceID, MotorType.kBrushed);
    m_followleftMotor = new CANSparkMax(followleftDeviceID, MotorType.kBrushed);
    m_leadrightMotor = new CANSparkMax(leadrightDeviceID, MotorType.kBrushed);
    m_followrightMotor = new CANSparkMax(followrightDeviceID, MotorType.kBrushed);
    
    // Set the encoders to 0


    // invert the left side motors
    m_leadrightMotor.setInverted(true);
  

    // Set the followers to follow the leaders
    m_followleftMotor.follow(m_leadleftMotor);
    m_followrightMotor.follow(m_leadrightMotor);

    

    // Initialize the robot
    m_myRobot = new DifferentialDrive(m_leadleftMotor, m_leadrightMotor);

    // Initialize joysticks
    m_ControllerMain = new XboxController(0);
    
  }

  @Override
  public void autonomousPeriodic(){
    timer.start();
    m_myRobot.tankDrive(0.25, 0.25);
    if (timer.hasElapsed(1)){
      m_myRobot.tankDrive(0, 0);
    }
  }

  

  @Override
  public void teleopPeriodic() {

    // Get Joystick value
    throttle = m_ControllerMain.getRawAxis(m_leftStickYID);
    difference = m_ControllerMain.getRawAxis(m_rightStickXID);

    
    // Get Trigger value
    leftTriggerMain = m_ControllerMain.getRawAxis(m_leftTriggerID);
    rightTriggerMain = m_ControllerMain.getRawAxis(m_rightTriggerID);

    
   
    SmartDashboard.putNumber("LTMain", leftTriggerMain);
    SmartDashboard.putNumber("RTMain", rightTriggerMain);
 

    // Deadband the sticks
    if (Math.abs(throttle) < 0.05) {
      throttle = 0;
    }
    if (Math.abs(difference) < 0.05) {
      difference = 0;
    }
    if (Math.abs(rightTriggerMain) < 0.05) {
      rightTriggerMain = 0;
    }

    // if RTMain is not pressed decrease the power of the motors
    if (rightTriggerMain == 0) {
      throttle = throttle * 0.7;
      difference = difference * 0.7;
      SmartDashboard.putBoolean("Turbo", false);
    }


    // Calculate the power for each motor
    leftMotors = throttle - difference;
    rightMotors = throttle + difference;
    SmartDashboard.putNumber("Throttle", throttle);
    SmartDashboard.putNumber("Difference", difference);

    // check if the power is greater than 1 or less than -1
    leftMotors = Math.max(-1, Math.min(leftMotors, 1));
    rightMotors = Math.max(-1, Math.min(rightMotors, 1));
    SmartDashboard.putNumberArray("Motors[L][R]", new double[] {leftMotors, rightMotors});
   

    // Drive the robot
    m_myRobot.tankDrive(leftMotors, rightMotors);
  }
}