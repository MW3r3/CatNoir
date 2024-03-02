package frc.robot;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;




public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private XboxController m_Controller;
  private double neovolt;

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
  private double leftTrigger;
  private double rightTrigger;

  // The CAN ID of the motor connected to the Spark Max
  private static final int leadleftDeviceID = 10;
  private static final int followleftDeviceID = 11;
  private static final int leadrightDeviceID = 12;
  private static final int followrightDeviceID = 13;
  private static final int shooterTopDeviceID = 14;
  private static final int shooterBottomDeviceID = 15;

  private CANSparkMax m_leadleftMotor;
  private CANSparkMax m_followleftMotor;
  private CANSparkMax m_leadrightMotor;
  private CANSparkMax m_followrightMotor;
  private CANSparkMax m_shooterBottomMotor;
  private CANSparkMax m_shooterTopMotor;

  @Override
  public void robotInit() {

    // Initialize the motors
    m_leadleftMotor = new CANSparkMax(leadleftDeviceID, MotorType.kBrushed);
    m_followleftMotor = new CANSparkMax(followleftDeviceID, MotorType.kBrushed);
    m_leadrightMotor = new CANSparkMax(leadrightDeviceID, MotorType.kBrushed);
    m_followrightMotor = new CANSparkMax(followrightDeviceID, MotorType.kBrushed);
    m_shooterBottomMotor = new CANSparkMax(shooterBottomDeviceID, MotorType.kBrushless);
    m_shooterTopMotor = new CANSparkMax(shooterTopDeviceID, MotorType.kBrushless);

    // top motor spins opposite to bottom motor
    m_shooterBottomMotor.follow(m_shooterTopMotor, true);


    // invert the left side motors
    m_leadrightMotor.setInverted(true);
  

    // Set the followers to follow the leaders
    m_followleftMotor.follow(m_leadleftMotor);
    m_followrightMotor.follow(m_leadrightMotor);
    

    // Initialize the robot
    m_myRobot = new DifferentialDrive(m_leadleftMotor, m_leadrightMotor);

    // Initialize joysticks
    m_Controller = new XboxController(0);
  }

  @Override
  public void teleopPeriodic() {
    neovolt = 0.0;

    // Get Joystick value
    throttle = m_Controller.getRawAxis(m_leftStickYID);
    difference = m_Controller.getRawAxis(m_rightStickXID);
    
    // Get Trigger value
    leftTrigger = m_Controller.getRawAxis(m_leftTriggerID);
    rightTrigger = m_Controller.getRawAxis(m_rightTriggerID);
 

    // Deadband the sticks
    if (Math.abs(throttle) < 0.05) {
      throttle = 0;
    }
    if (Math.abs(difference) < 0.05) {
      difference = 0;
    }
    if (Math.abs(leftTrigger) < 0.05) {
      leftTrigger = 0;
    }
    if (Math.abs(rightTrigger) < 0.05) {
      rightTrigger = 0;
    }

    throttle = throttle * 0.7;
    difference = difference * 0.7;

    // Calculate the power for each motor
    leftMotors = throttle - difference;
    rightMotors = throttle + difference;

    // check if the power is greater than 1 or less than -1
    leftMotors = Math.max(-1, Math.min(leftMotors, 1));
    rightMotors = Math.max(-1, Math.min(rightMotors, 1));

    // Drive BottomShooterMotor accordingly to right trigger
    if(rightTrigger > 0) {
      neovolt = 12.0;
      m_shooterTopMotor.setVoltage(neovolt);
    }

    
    

    SmartDashboard.putNumber("Throttle", throttle);
    SmartDashboard.putNumber("Difference", difference);
    SmartDashboard.putNumber("LT", leftTrigger);
    SmartDashboard.putNumber("RT", rightTrigger);



    // Drive the robot
    m_myRobot.tankDrive(leftMotors, rightMotors);
  }
}