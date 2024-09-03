// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterSub;

public class ShooterCmd extends Command {
  private final ShooterSub shooter;
  private final XboxController ps5Controller = new XboxController(1);

  
  /** Creates a new ShooterCmd. */
  public ShooterCmd(ShooterSub shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*ShuffleboardTab tab = Shuffleboard.getTab("Shooter Direction");
    tab.add("Shoot", buttonPressed == false);
    tab.add("take", buttonPressed == true);
    */

    /*     
    shooter.shoot(xboxController.getRightTriggerAxis());          //Shoot for speaker


    shooter.shootWithBool(-0.2, xboxController.getYButton());      //run reverse direction shooter
    shooter.shootWithBool(0.35, xboxController.getRightBumper());  //Shoot for Amp


    shooter.runBottomShooter(xboxController.getLeftTriggerAxis());  //Pass to shooter for speaker

    shooter.bottomShooterRunWBool(0.5, xboxController.getLeftBumper()); //run bottom shooter  for amp
    shooter.bottomShooterRunWBool(-0.2, xboxController.getBButton());   //run reverse direction bottom shooter

    */

    shooter.shoot(ps5Controller.getRightTriggerAxis());          //Shoot for speaker

    shooter.shootWithBool(-0.2, ps5Controller.getYButton());      //run reverse direction shooter
    shooter.shootWithBool(0.35, ps5Controller.getRightBumper());  //Shoot for Amp

    shooter.runBottomShooter(ps5Controller.getLeftTriggerAxis());  //Pass to shooter for speaker

    shooter.bottomShooterRunWBool(0.5, ps5Controller.getLeftBumper()); //run bottom shooter  for amp
    shooter.bottomShooterRunWBool(-0.2, ps5Controller.getBButton());   //run reverse direction bottom shooter
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}