// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSub extends SubsystemBase {


  private final CANSparkMax m_leftBottomShooter = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax m_rightBottomShooter = new CANSparkMax(8, MotorType.kBrushless);
  private final TalonFX m_leftShooter = new TalonFX(5,"rio");
  private final TalonFX m_rightShooter = new TalonFX(6,"rio");

  private final Servo servoCam = new Servo(4);
  private final Servo servoCam2 = new Servo(3);
  
  Orchestra m_orchestra = new Orchestra();

  boolean servoPressed;

  // Creates UsbCamera and MjpegServer [1] and connects them

  /** Creates a new ShooterSub. */
  public ShooterSub() {
    m_leftShooter.setInverted(true);
    m_rightShooter.setInverted(false);
    m_leftBottomShooter.setInverted(false);
    m_rightBottomShooter.setInverted(true);
    m_rightShooter.setNeutralMode(NeutralModeValue.Brake);
    m_leftShooter.setNeutralMode(NeutralModeValue.Brake);

    //chooseMusic("neverGonnaGiveYouUp.chrp");
    servoPressed = false;

    //("DrDre", "DrDreStillDre.chrp");
    //("Never Gonna Give", "neverGonnaGiveYouUp.chrp");  
    //("Mario Bros", "output.chrp");  

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(double trigger) {
   
    m_leftShooter.set(-trigger * 0.91);
   m_rightShooter.set(-trigger * 0.91);

  }

  public void shootWithBool(double vel, boolean button) {
    if (button) {
      m_leftShooter.set(-vel);
      m_rightShooter.set(-vel);
    }
  }

  public void bottomShooterRunWBool(double vel, boolean button) {
    if (button) {
      m_leftBottomShooter.set(vel);
      m_rightBottomShooter.set(vel);

    }
  }

  public void stop() {
    m_leftShooter.stopMotor();
  }

  public void runBottomShooter(double trigger) {
    m_leftBottomShooter.set(trigger);
    m_rightBottomShooter.set(trigger);
  }

  public double shootingSpeed() {
    return ((m_leftShooter.get() + m_rightShooter.get()) / 2);
  }

  public double buttomShooterSpeed() {
    return (m_leftBottomShooter.get()+m_rightBottomShooter.get()) / 2;
  }

  public void setServoAngle(int angle) {
    servoCam.setAngle(angle/2);
    servoCam2.setAngle((180-(angle/2)));
  }

  public void passNote(boolean button){
    m_leftBottomShooter.set(0.4);
    m_rightBottomShooter.set(0.4);
  }

  public void chooseMusic(String filename) {
    // Add a single device to the orchestra
    m_orchestra.addInstrument(m_leftShooter);
    m_orchestra.addInstrument(m_rightShooter);

    // Attempt to load the chrp
    m_orchestra.loadMusic(filename);
    m_orchestra.play();
  }

}