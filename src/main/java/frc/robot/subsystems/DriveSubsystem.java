// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  PWMSparkMax leftMotor = new PWMSparkMax(DriveConstants.leftMotorPWM);
  //PWMSparkMax sleftMotor = new PWMSparkMax(DriveConstants.leftMotorPWM);
  PWMSparkMax rightMotor = new PWMSparkMax(DriveConstants.rightMotorPWM);

  //MotorController leftMotor = new MotorControllerGroup(mleftMotor, sleftMotor);
  
  DifferentialDrive mDrive = new DifferentialDrive(leftMotor, rightMotor);
  AnalogGyro mGyro = new AnalogGyro(0);
  public DriveSubsystem() {
    rightMotor.setInverted(true);
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    mDrive.arcadeDrive(xSpeed, zRotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    mDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public double getAngle(){
    return mGyro.getAngle();
  }

  public void zeroHeading() {
    mGyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro Angle", getAngle());
  }
}
