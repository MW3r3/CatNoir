package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private DifferentialDrive m_myRobot;
  private XboxController m_ControllerMain;

  // Trigger variables
  private static final int m_leftStickYID = 1;
  private static final int m_rightStickXID = 4;
  private static final int m_rightTriggerID = 3;
  private static final int m_leftTriggerID = 2;

  // Motor power variables
  private double throttle;
  private double difference;
  private double leftMotors;
  private double rightMotors;
  private double shooter;

  private double rightTriggerMain;
  private double leftTriggerMain;
  private Timer timer = new Timer();

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
  private CANSparkMax m_shooterTopMotor;
  private CANSparkMax m_shooterBottomMotor;

  private static final MotorType Brushed = MotorType.kBrushed;

  // private static final MotorType Brushless = MotorType.kBrushless;

  @Override
  public void robotInit() {
    m_leadleftMotor = new CANSparkMax(leadleftDeviceID, Brushed);
    m_followleftMotor = new CANSparkMax(followleftDeviceID, Brushed);
    m_leadrightMotor = new CANSparkMax(leadrightDeviceID, Brushed);
    m_followrightMotor = new CANSparkMax(followrightDeviceID, Brushed);
    m_shooterTopMotor = new CANSparkMax(shooterTopDeviceID, Brushed);
    m_shooterBottomMotor = new CANSparkMax(shooterBottomDeviceID, Brushed);

    m_followleftMotor.follow(m_leadleftMotor);
    m_followrightMotor.follow(m_leadrightMotor, true);
    m_shooterBottomMotor.follow(m_shooterTopMotor);

    m_myRobot = new DifferentialDrive(m_leadleftMotor, m_leadrightMotor);

    m_ControllerMain = new XboxController(0);
  }

  @Override
  public void autonomousPeriodic() {
    timer.start();
    m_myRobot.tankDrive(0.25, 0.25);
    if (timer.hasElapsed(2)) {
      m_myRobot.tankDrive(0, 0);
    }
  }

  @Override
  public void autonomousExit() {
    timer.reset();
    timer.stop();
  }

  @Override
  public void teleopPeriodic() {
    // Get Joystick value
    throttle = m_ControllerMain.getRawAxis(m_leftStickYID);
    difference = m_ControllerMain.getRawAxis(m_rightStickXID);

    // Get Trigger value
    rightTriggerMain = m_ControllerMain.getRawAxis(m_rightTriggerID);
    leftTriggerMain = m_ControllerMain.getRawAxis(m_leftTriggerID);

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
    if (Math.abs(leftTriggerMain) < 0.05) {
      leftTriggerMain = 0;
    }

    // if RTMain is not pressed decrease the power of the motors
    if (rightTriggerMain == 0) {
      throttle = throttle * 0.7;
      difference = difference * 0.7;
    }

    if (rightTriggerMain < 0) {
      m_shooterTopMotor.set(rightTriggerMain);
    } else {
      m_shooterTopMotor.set(-leftTriggerMain);
    }

    // Calculate the power for each motor
    leftMotors = throttle - difference;
    rightMotors = throttle + difference;

    // check if the power is greater than 1 or less than -1
    leftMotors = Math.max(-1, Math.min(leftMotors, 1));
    rightMotors = Math.max(-1, Math.min(rightMotors, 1));

    // Drive the robot
    m_myRobot.tankDrive(leftMotors, rightMotors);
  }
}
