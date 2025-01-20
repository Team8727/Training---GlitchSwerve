// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.Constants.kIntake.kPivot;
import frc.robot.Constants.kIntake.kPivot.IntakePosition;

public class IntakeCmds extends Command {
  IntakePivot m_intakePivot;
  IntakeRollers m_intakeRollers;
  
  /** Creates a new IntakeCommand. */
  public IntakeCmds(IntakePivot intakePivot, IntakeRollers intakeRollers) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakePivot = intakePivot;
    m_intakeRollers = intakeRollers;
    addRequirements(intakePivot, intakeRollers);
  }

  public Command intake() {
    return Commands.deadline(
          m_intakeRollers.intake(), m_intakePivot.setIntakePosition(kPivot.IntakePosition.DEPLOYED)) //.onlyWhile(() -> m_intakePivot.isAtPosition == false))
      .andThen(Commands.race(
              m_intakePivot.waitUntilIsAtPosition(IntakePosition.HOME), 
              m_intakePivot.setIntakePosition(kPivot.IntakePosition.HOME)));
  }
}