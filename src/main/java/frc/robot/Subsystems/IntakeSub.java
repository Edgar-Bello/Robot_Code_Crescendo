// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {
  private final CANSparkMax m_intake = new CANSparkMax(11, MotorType.kBrushed);
  /** Creates a new IntakeSub. */
  public IntakeSub() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(boolean buttonPos, boolean buttonNeg, double vel) {
    if (buttonPos) {
      m_intake.set(vel);
    }
    else if (buttonNeg) {
      m_intake.set(-vel);
    }
    else {
      m_intake.stopMotor();
    }
  }

  public void take(boolean button, double vel) {
    if (button) {
      m_intake.set(vel);
    }
    else {
      m_intake.stopMotor();
    }
  }

  public void stop() {
    m_intake.stopMotor();
  }

  public boolean isTaking() {
    return (m_intake.get() > 0.1);
  }

}