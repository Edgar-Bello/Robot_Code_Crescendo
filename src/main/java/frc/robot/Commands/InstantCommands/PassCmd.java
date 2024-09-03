// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.InstantCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterSub;

public class PassCmd extends Command {
  private final ShooterSub m_shooter;
  private final Timer m_timer2 = new Timer();
  /** Creates a new PassCmd. */
  public PassCmd(ShooterSub m_shooter) {
    this.m_shooter = m_shooter;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer2.reset();
    m_timer2.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.runBottomShooter(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer2.get() > 0.7) {
      return true;
    }
    else {
      return false;
    }
  }
}