// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveSystem extends SubsystemBase {
  
  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule backLeft;
  SwerveModule backRight;

  SwerveModule[] modulesArray;

  SwerveDriveKinematics kinematics;

  public SwerveSystem() {

    modulesArray = new SwerveModule[4];

    frontLeft = new SwerveModule(0, 40, 1, -60);
    frontRight = new SwerveModule(22, 8, 15, 223);
    backLeft = new SwerveModule(61, 60, 51, 84);
    backRight = new SwerveModule(20, 30, 21, -105);

    modulesArray[0] = frontLeft;
    modulesArray[1] = frontRight;
    modulesArray[2] = backLeft;
    modulesArray[3] = backRight;

    kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.DRIVE_OFFSET, Constants.DRIVE_OFFSET),
      new Translation2d(Constants.DRIVE_OFFSET, -Constants.DRIVE_OFFSET),
      new Translation2d(-Constants.DRIVE_OFFSET, Constants.DRIVE_OFFSET),
      new Translation2d(-Constants.DRIVE_OFFSET, -Constants.DRIVE_OFFSET)
      );
  }

  public void stop(){
    for(int i = 0; i < 4; i++){
      modulesArray[i].stop();
    } 
  }

  public void move(double drive, double rotaion){
    for(int i = 0; i < 4; i++){
      modulesArray[i].move(drive, rotaion);
    }
  }

  public void setDesiredStates(SwerveModuleState[] states){
    for(int i = 0; i < 4; i++){
      modulesArray[i].setDesiredState(states[i]);
    }
  }

  public void drive(double speedY, double speedX, double rotation){
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(speedX, speedY, Math.toRadians(rotation)));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 4.4196);
    setDesiredStates(states);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("front left pos", modulesArray[0].getHeadingDegrees());
    SmartDashboard.putNumber("front right pos", modulesArray[1].getHeadingDegrees());
    SmartDashboard.putNumber("back left pos", modulesArray[2].getHeadingDegrees());
    SmartDashboard.putNumber("back right pos", modulesArray[3].getHeadingDegrees());

    SmartDashboard.putNumber("front left abs encoder", modulesArray[0].getAbsEncoder());
    SmartDashboard.putNumber("front right abs encoder", modulesArray[1].getAbsEncoder());
    SmartDashboard.putNumber("back left abs encoder", modulesArray[2].getAbsEncoder());
    SmartDashboard.putNumber("back right abs encoder", modulesArray[3].getAbsEncoder());

    SmartDashboard.putNumber("front left steer sensoer pos", modulesArray[0].steerSensoerPos());
    SmartDashboard.putNumber("front right steer sensoer pos", modulesArray[1].steerSensoerPos());
    SmartDashboard.putNumber("back left steer sensoer pos", modulesArray[2].steerSensoerPos());
    SmartDashboard.putNumber("back right steer sensoer pos", modulesArray[3].steerSensoerPos());
  }
}
