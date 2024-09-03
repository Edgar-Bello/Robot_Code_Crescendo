
package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HookSub extends SubsystemBase {
  private final CANSparkMax m_hookMotor1 = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax m_hookMotor2 = new CANSparkMax(9, MotorType.kBrushless);

  /** Creates a new HookingSub. */
  public HookSub() {
    m_hookMotor1.setInverted(false);
    m_hookMotor2.setInverted(true);
  }

  public void setBrakeMode(){
    m_hookMotor1.setIdleMode(IdleMode.kBrake);
    m_hookMotor2.setIdleMode(IdleMode.kBrake);

  }

  public void hook(double vel, boolean button) {
    if (button) {
      m_hookMotor1.set(-vel);
      m_hookMotor2.set(vel);
    }

    SmartDashboard.putNumber("Hooking Vel", hookingVel());
    SmartDashboard.putBoolean("Hooking value", hookingVel() != 0);
  }

  public void hookWDouble(double trigger) {
    if (trigger < 0.4 && trigger > -0.4) {
      m_hookMotor1.set(0);
      m_hookMotor2.set(0);
    } else {
      m_hookMotor1.set(trigger);
      m_hookMotor2.set(trigger);
    }
  }

  public double hookingVel() {
    return (m_hookMotor1.get() + m_hookMotor2.get()) /2;
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}