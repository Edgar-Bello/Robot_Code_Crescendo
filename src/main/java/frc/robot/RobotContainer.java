// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.DrivetrainCmd;
import frc.robot.Commands.HookingCmd;
import frc.robot.Commands.IntakeCmd;
import frc.robot.Commands.ShooterCmd;
import frc.robot.Commands.InstantCommands.ShootCmd;
import frc.robot.Commands.InstantCommands.TakeCmd;
import frc.robot.Subsystems.DrivetrainSub;
import frc.robot.Subsystems.HookSub;
import frc.robot.Subsystems.IntakeSub;
import frc.robot.Subsystems.ShooterSub;

public class RobotContainer {
  private final DrivetrainSub m_drivetrain = new DrivetrainSub();
  private final ShooterSub m_shooter = new ShooterSub();
  private final IntakeSub m_intake = new IntakeSub();
  private final HookSub m_hookMotors = new HookSub();


  ShuffleboardTab s_teleop = Shuffleboard.getTab("TeleopTab");
  GenericEntry s_rightDistance = s_teleop.add("Right Distane", 0).getEntry();
  GenericEntry s_leftDistance = s_teleop.add("Left Distance", 0).getEntry();
  GenericEntry s_gyroHeading = s_teleop.add("Gyro Heading", 0).getEntry();
  GenericEntry s_leftVel = s_teleop.add("Left Velocity", 0).getEntry();
  GenericEntry s_rightVel = s_teleop.add("Right Velocity", 0).getEntry();
  GenericEntry s_lowSpeed = s_teleop.add("Low Speed", false).getEntry();
  GenericEntry s_MidSpeed = s_teleop.add("Mid Speed", false).getEntry();
  GenericEntry s_MaxSpeed = s_teleop.add("Max Speed", false).getEntry();

  GenericEntry s_shooterVel = s_teleop.add("Shooter Speed", 0).getEntry();
  GenericEntry s_shooterIsActve = s_teleop.add("Shooter Active", false).getEntry();
  GenericEntry s_bottomShooterVel = s_teleop.add("B Shooter Vel", 0).getEntry();
  GenericEntry s_bottomShooterIsActive = s_teleop.add("B Shooter Active", false).getEntry();

  GenericEntry s_intakeIsActive = s_teleop.add("Intake Active", false).getEntry();

  GenericEntry s_hookingMotorsVel = s_teleop.add("Hook Vel", 0).getEntry();
  GenericEntry s_hookingMotorsActive = s_teleop.add("Hook Active", false).getEntry();
  
  UsbCamera usbCamera = new UsbCamera("USB Camera 0", 1);


  public RobotContainer() {
    configureBindings();
    CameraServer.startAutomaticCapture();
    //NamedCommands.registerCommand("grab", new InstantCommand(()-> m_intake));
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(new DrivetrainCmd(m_drivetrain));
    m_shooter.setDefaultCommand(new ShooterCmd(m_shooter));
    m_intake.setDefaultCommand(new IntakeCmd(m_intake));
    m_hookMotors.setDefaultCommand(new HookingCmd(m_hookMotors));  
  }

  public Command getAutonomousCommand() {
    return new ParallelCommandGroup(new TakeCmd(m_intake), new ShootCmd(m_shooter));
  }


  public void getTeleopValues() {
    s_rightDistance.setDouble(m_drivetrain.getRightEncoderPosition());
    s_leftDistance.setDouble(m_drivetrain.getLeftEncoderPosition());
    s_rightVel.setDouble(m_drivetrain.getRightEncoderVelocity());
    s_leftVel.setDouble(m_drivetrain.getLeftEncoderVelocity());
    s_gyroHeading.setDouble(m_drivetrain.getHeading());
    s_lowSpeed.setBoolean(m_drivetrain.getButtonPressedOnce());
    s_MidSpeed.setBoolean(m_drivetrain.getButtonPressedTwice());
    s_MaxSpeed.setBoolean(m_drivetrain.getButtonPressedOnce() == false && m_drivetrain.getButtonPressedTwice() == false);

    s_shooterIsActve.setBoolean(m_shooter.shootingSpeed() != 0);
    s_shooterVel.setDouble(m_shooter.shootingSpeed());
    s_bottomShooterIsActive.setBoolean(m_shooter.buttomShooterSpeed() != 0);
    s_bottomShooterVel.setDouble(m_shooter.buttomShooterSpeed());

    s_intakeIsActive.setBoolean(m_intake.isTaking());

    s_hookingMotorsActive.setBoolean(m_hookMotors.hookingVel() != 0);
    s_hookingMotorsVel.setDouble(m_hookMotors.hookingVel());
    
    //SmartDashboard.putData("autoChooser", autoChooser);


  }
}