// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstrants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  PWMSparkMax armMotor = new PWMSparkMax(ArmConstrants.armMotorPWM);
  public ArmSubsystem() {}

  public void armMotor(double speed) {
    armMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
