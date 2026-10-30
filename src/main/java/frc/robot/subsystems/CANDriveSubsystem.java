// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
//import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Drive;

import static frc.robot.Constants.DriveConstants.*;

//import edu.wpi.first.math.controller.PIDController;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;
  //private AHRS m_navx;
  private final PIDController m_headingPID;

  private final DifferentialDrive drive;

  private static final double FORWARD_SPEED = 0.5;   // Base forward speed (0.0 to 1.0)
  private static final double kP = 0.015;            // Proportional gain for heading correction
  private static final double kI = 0.0;              // Integral (usually 0 for short straight drives)
  private static final double kD = 0.001;            // Derivative

  AHRS m_navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

  @SuppressWarnings("removal")
  public CANDriveSubsystem() {
    
    //try{
    
    //AHRS m_navx = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
    //initializeNavX();

    
    //}
    //catch (RuntimeException ex) {
      //DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    //}
  
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);
  
    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);
    
    
    
    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);
  
    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    m_headingPID = new PIDController(kP, kI, kD);
    m_headingPID.setTolerance(2.0); // Degrees tolerance for "on target"

    // Since yaw wraps -180..180, enable continuous input for smooth PID
    m_headingPID.enableContinuousInput(-180.0, 180.0);

    // This command requires the drivetrain (in a real subsystem you'd addRequirements)
    // For standalone example we assume the caller handles it.
  }

  public void zeroHeading() {
      m_navx.reset();
  }

  public double getYawDegrees() {
      return m_navx.getYaw();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getYawDegrees());
  }

  public boolean isGyroCalibrating() {
    return m_navx.isCalibrating();
  }

  public double getWorldAccelX() {
    return m_navx.getWorldLinearAccelX();
  }

  public double getWorldAccelY() {
    return m_navx.getWorldLinearAccelY();
  }

  public double getRawAccelX() {
    return m_navx.getRawAccelX();
  }

 public double getRawAccelY() {
    return m_navx.getRawAccelY();
  }

  @Override
  public void periodic() {
  }
  
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void driveStraight(double forwardSpeed){
        //double FORWARD_SPEED = forwardSpeed;
        double currentYaw = m_navx.getYaw();
        double turnPower = m_headingPID.calculate(currentYaw);
        double leftSpeed = FORWARD_SPEED + turnPower; // inversed the + , - before
        double rightSpeed = FORWARD_SPEED - turnPower;// same here but opposite
        SmartDashboard.putNumber("currentYaw", currentYaw);
        SmartDashboard.putNumber("turnPower", turnPower);
        drive.tankDrive(leftSpeed, rightSpeed);

  }

  public void initializeNavX(){
     // Zero everything at the start of the drive
        m_navx.zeroYaw();
        m_navx.resetDisplacement();

        // Target heading is now "straight" (0 degrees relative to starting orientation)
        m_headingPID.setSetpoint(0.0);

        drive.tankDrive(0,0);  
  }


  public boolean checkFinished(double target){
    double m_targetDistanceMeters = target;
    double dispX = m_navx.getDisplacementX();
        double dispY = m_navx.getDisplacementY();
        double distanceTraveled = Math.sqrt(dispX*dispX+dispY*dispY);
        SmartDashboard.putNumber("distanceTraveled",distanceTraveled);
        SmartDashboard.putNumber("Target Distance",m_targetDistanceMeters);
        return distanceTraveled >= m_targetDistanceMeters;
  }

  public void initDefaultCommand(Drive drive2) {
    // TODO Auto-generated method stub
    setDefaultCommand(drive2);
    throw new UnsupportedOperationException("Unimplemented method 'setDefaultCommand'");
  }

  
}

