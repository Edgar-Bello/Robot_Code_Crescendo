// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSub extends SubsystemBase {

  private final CANSparkMax m_leftMaster = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_leftFollower = new CANSparkMax(4, MotorType.kBrushed);
  private final CANSparkMax m_rightMaster = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_rightFollower = new CANSparkMax(2, MotorType.kBrushed);
  private final DifferentialDrive m_drivetrain = new DifferentialDrive(m_leftMaster, m_rightMaster);
  private final static AHRS navx = new AHRS(SPI.Port.kMXP);
  private final Encoder m_leftEncoder = new Encoder(6, 5, false, EncodingType.k2X);
  private final Encoder m_rightEncoder = new Encoder(4, 3,true, EncodingType.k2X);
  boolean buttonPressedOnce;
  boolean buttonPressedTwice;
  
  // Creating my kinematics object: track width of 27 inches
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.kTrackWidthMeters);
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(navx.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

  double linearVelocity = 3.0; //3.0
  double angularVelocity = 6.28; //6.28

  public Field2d field = new Field2d();


  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {
    setSmartLimit(50);
    m_rightMaster.setInverted(true);
    m_rightFollower.setInverted(true);

    m_leftFollower.follow(m_leftMaster);
    m_rightFollower.follow(m_rightMaster);


    buttonPressedOnce = false;
    buttonPressedTwice = false;


    m_leftEncoder.setDistancePerPulse(Constants.kConvertionFactor);
    m_rightEncoder.setDistancePerPulse(Constants.kConvertionFactor);
    
    field.setRobotPose(0.0, 4.0, getRotation2d());

    setBrakeMode();

    resetEncoders();
    
  }

  @Override
  public void periodic() {
    m_odometry.update(getRotation2d(), m_leftEncoder.getDistance(),
    m_rightEncoder.getDistance());

    field.setRobotPose(getPose());
    SmartDashboard.putData("field", field);
    SmartDashboard.putData("Left Encoder", getLeftEncoder());
    SmartDashboard.putData("Right Encoder", getRightEncoder());
    SmartDashboard.putNumber("pose2d", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("gyro", getHeading());

    // This method will be called once per scheduler run
  }

  public void setCoastMode() {
    m_leftMaster.setIdleMode(IdleMode.kCoast);
    m_leftFollower.setIdleMode(IdleMode.kCoast);
    m_rightMaster.setIdleMode(IdleMode.kCoast);
    m_rightFollower.setIdleMode(IdleMode.kCoast);
  }


  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void driveTeleop(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  public void limitSpeed(double MaxSpeed, double MidSpeed, double LowSpeed, boolean button) {
    if (button) {
      if (buttonPressedTwice == true) {
        m_drivetrain.setMaxOutput(MaxSpeed);
        buttonPressedTwice = false;
      }
      else {
        if (buttonPressedOnce == true) {
          m_drivetrain.setMaxOutput(MidSpeed);
          buttonPressedOnce = false;
          buttonPressedTwice = true;
        }
        else {
          m_drivetrain.setMaxOutput(LowSpeed);
          buttonPressedOnce = true;
        }
      }
    }
  }

  public double getRightEncoderPosition() {
    return m_rightEncoder.getDistance();
    //CHECK IF NEEDS TO BE INVERTED
  }

  public double getLeftEncoderPosition() {
    return m_leftEncoder.getDistance();
    //CHECK IF NEEDS TO BE INVERTED
  }

  public double getRightEncoderVelocity() {
    return m_rightEncoder.getRate();
  }

    public double getLeftEncoderVelocity() {
    return m_leftEncoder.getRate();
  }

  public double getTurnRate() {
    return navx.getRate();
  }
  public double getHeading() {
    return getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  public Rotation2d getRotation2d() {
    return navx.getRotation2d();
  }

  public double getAverageEncoderDistance() {
    return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
  }

  public double getAverageEncoderVel() {
    return ((getLeftEncoderVelocity() + getRightEncoderVelocity()) / 2.0);
  }

  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    m_drivetrain.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    navx.reset();
  }

  public AHRS getGyro() {
    return navx;
  }

  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  public void setSmartLimit(int MaxOutput) {
    m_leftMaster.setSmartCurrentLimit(MaxOutput);
    m_rightMaster.setSmartCurrentLimit(MaxOutput);
    m_leftFollower.setSmartCurrentLimit(MaxOutput);
    m_rightFollower.setSmartCurrentLimit(MaxOutput);
  }

  public boolean getButtonPressedOnce() {
    return buttonPressedOnce;
  }

  public boolean getButtonPressedTwice() {
    return buttonPressedTwice;
  }

  public void setBrakeMode() {
    m_leftFollower.setIdleMode(IdleMode.kBrake);
    m_rightMaster.setIdleMode(IdleMode.kBrake);
    m_leftMaster.setIdleMode(IdleMode.kBrake);
    m_rightFollower.setIdleMode(IdleMode.kBrake);
  }
}