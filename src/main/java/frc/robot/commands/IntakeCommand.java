// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */

  IntakeSubsystem intakeSub;
  HopperSubsystem hopperSub;

  public IntakeCommand(IntakeSubsystem i, HopperSubsystem h) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSub = i;
    hopperSub = h;
    addRequirements(i, h);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSub.setIntakeMotor(.7);
    hopperSub.setMotors(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    intakeSub.setIntakeMotor(0);
    hopperSub.setMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
