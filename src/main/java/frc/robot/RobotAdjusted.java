/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import java.util.ArrayList;
import java.util.Collections;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotAdjusted extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  //private Joystick m_rightStick;
  
  private CANSparkMax m_leftFrontDriveMotor;
  private CANSparkMax m_rightFrontDriveMotor;
  private CANSparkMax m_leftBackDriveMotor;
  private CANSparkMax m_rightBackDriveMotor;
  private CANSparkMax m_leftFrontTurningMotor;
  private CANSparkMax m_rightFrontTurningMotor;
  private CANSparkMax m_leftBackTurningMotor;
  private CANSparkMax m_rightBackTurningMotor;


  private static final int leftFrontDriveMotorID = 2;
  private static final int rightFrontDriveMotorID = 4;
  private static final int leftBackDriveMotorID = 8;
  private static final int rightBackDriveMotorID = 6;
  private static final int leftFrontTurningMotorID = 1;
  private static final int rightFrontTurningMotorID = 3;
  private static final int leftBackTurningMotorID = 7;
  private static final int rightBackTurningMotorID = 5;

  private static final int leftFrontCANCoderID = 11;
  private static final int rightFrontCANCoderID = 12;
  private static final int leftBackCANCoderID = 14;
  private static final int rightBackCANCoderID = 13;

  private static final CANcoder leftFrontTurningEncoder = new CANcoder(leftFrontCANCoderID);
  private static final CANcoder rightFrontTurningEncoder = new CANcoder(rightFrontCANCoderID);
  private static final CANcoder leftBackTurningEncoder = new CANcoder(leftBackCANCoderID);
  private static final CANcoder rightBackTurningEncoder = new CANcoder(rightBackCANCoderID);

  final double TURNING_GEAR_RATIO = 150/7;

  ArrayList<CANSparkMax> motors;

  // Define the locations of the swerve modules relative to the robot center
  private final Translation2d m_frontLeftLocation = new Translation2d(0.5, 0.5);
  private final Translation2d m_frontRightLocation = new Translation2d(0.5, -0.5);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.5, 0.5);
  private final Translation2d m_backRightLocation = new Translation2d(-0.5, -0.5);

  // Create the kinematics object
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

   @Override
   public void robotInit() {
     /**
      * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
      * 
      * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
      * first parameter
      * 
      * The motor type is passed as the second parameter. Motor type can either be:
      *  com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
      *  com.revrobotics.CANSparkLowLevel.MotorType.kBrushed
      * 
      * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
      * these parameters to match your setup
      */
     m_leftFrontDriveMotor = new CANSparkMax(leftFrontDriveMotorID, MotorType.kBrushless);
     m_rightFrontDriveMotor = new CANSparkMax(rightFrontDriveMotorID, MotorType.kBrushless);
     m_leftBackDriveMotor = new CANSparkMax(leftBackDriveMotorID, MotorType.kBrushless);
     m_rightBackDriveMotor = new CANSparkMax(rightBackDriveMotorID, MotorType.kBrushless);
     m_leftFrontTurningMotor = new CANSparkMax(leftFrontTurningMotorID, MotorType.kBrushless);
     m_rightFrontTurningMotor = new CANSparkMax(rightFrontTurningMotorID, MotorType.kBrushless);
     m_leftBackTurningMotor = new CANSparkMax(leftBackTurningMotorID, MotorType.kBrushless);
     m_rightBackTurningMotor = new CANSparkMax(rightBackTurningMotorID, MotorType.kBrushless);
   
     /**
      * The RestoreFactoryDefaults method can be used to reset the configuration parameters
      * in the SPARK MAX to their factory default state. If no argument is passed, these
      * parameters will not persist between power cycles
      */
     m_leftFrontDriveMotor.restoreFactoryDefaults();
     m_rightFrontDriveMotor.restoreFactoryDefaults();
     m_leftBackDriveMotor.restoreFactoryDefaults();
     m_rightBackDriveMotor.restoreFactoryDefaults();
     m_leftFrontTurningMotor.restoreFactoryDefaults();
     m_rightFrontTurningMotor.restoreFactoryDefaults();
     m_leftBackTurningMotor.restoreFactoryDefaults();
     m_rightBackTurningMotor.restoreFactoryDefaults();

     motors = new ArrayList<CANSparkMax>();
     motors.add(m_leftFrontDriveMotor);
     motors.add(m_rightFrontDriveMotor);
     motors.add(m_leftBackDriveMotor);
     motors.add(m_rightBackDriveMotor);
     motors.add(m_leftFrontTurningMotor);
     motors.add(m_rightFrontTurningMotor);
     motors.add(m_leftBackTurningMotor);
     motors.add(m_rightBackTurningMotor);
   
     m_myRobot = new DifferentialDrive(m_leftFrontDriveMotor, m_rightFrontDriveMotor);
   
     m_leftStick = new Joystick(0);
     //m_rightStick = new Joystick(1);
   }

   public double getAdjustedEncoderPosition(RelativeEncoder encoder){
    double adjusted_angle = encoder.getPosition();
    adjusted_angle*= TURNING_GEAR_RATIO;
    adjusted_angle = adjusted_angle % 360;
    return adjusted_angle;
   }


   public void moveMotorToPosition(CANSparkMax motor, double targetPosition) {
    PIDController pidController = new PIDController(0.1, 0.0, 0.0);
    //motor.getEncoder().setPosition(targetPosition);
    RelativeEncoder encoder = motor.getEncoder();

    motor.set(pidController.calculate(encoder.getPosition(), targetPosition));
    // encoder.setPosition(0.5);
    System.out.println(getAdjustedEncoderPosition(encoder));
  }

  public void swerveDrive(double forward, double strafe, double rotation) {
    // Convert joystick inputs to chassis speeds
    var chassisSpeeds = new ChassisSpeeds(forward, strafe, rotation);

    // Calculate the desired states for each swerve module
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);

    // Normalize wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1.0);

    // Set each module's speed and angle
    setModuleState(m_leftFrontDriveMotor, m_leftFrontTurningMotor, moduleStates[0]);
    setModuleState(m_rightFrontDriveMotor, m_rightFrontTurningMotor, moduleStates[1]);
    setModuleState(m_leftBackDriveMotor, m_leftBackTurningMotor, moduleStates[2]);
    setModuleState(m_rightBackDriveMotor, m_rightBackTurningMotor, moduleStates[3]);
  }

  private void setModuleState(CANSparkMax driveMotor, CANSparkMax turningMotor, SwerveModuleState state) {
    driveMotor.set(state.speedMetersPerSecond);
    moveMotorToPosition(turningMotor, state.angle.getDegrees());
  }

  @Override
  public void teleopPeriodic() {
    if (m_leftStick.getRawButton(1)) {
      System.out.println("Resetting encoders");
      for (CANSparkMax motor : motors) {
        motor.getEncoder().setPosition(0);
      }
    }

    double forward = -m_leftStick.getY();
    double strafe = m_leftStick.getX();
    double rotation = m_leftStick.getTwist(); // Assuming twist for rotation

    swerveDrive(forward, strafe, rotation);
  }
}