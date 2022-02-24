// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  CANSparkMax rightClimb = new CANSparkMax(Constants.rightClimb,MotorType.kBrushless);
  CANSparkMax leftClimb = new CANSparkMax(Constants.leftClimb,MotorType.kBrushless);

  public ClimbSubsystem() {
    rightClimb.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 60000);
    leftClimb.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 60000);
    rightClimb.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 60000);
    leftClimb.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 60000);
  }

  public void setClimb(double x)
  {
    rightClimb.set(x);
    leftClimb.set(x);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
