// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.kIntake.kPivot;
import frc.robot.Constants.kIntake.kPivot.IntakePosition;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utilities.SparkConfigurator.LogData;
import frc.robot.utilities.SparkConfigurator.Sensors;
import monologue.Annotations.Log;
import monologue.Logged;
import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;



public class IntakePivot extends SubsystemBase implements Logged {
  /** Creates a new IntakePivot. */
  public final CANSparkMax pivotMotor;
  public final Encoder pivotMotorEncoder;

  private final ArmFeedforward pivotFF;
  private final ProfiledPIDController profiledPIDController;
  private final TrapezoidProfile.Constraints constraints =
      new Constraints(kPivot.maxVel, kPivot.maxAccel);
  private final TrapezoidProfile.State goal;
  private final TrapezoidProfile.State currentSetpoint;
  private IntakePosition goalPosition = IntakePosition.HOME;

  private ShuffleboardTab tab = Shuffleboard.getTab("Intake Pivot");

  public IntakePivot() {

    pivotMotor = 
        getSparkMax(kPivot.pivotMotorID, CANSparkLowLevel.MotorType.kBrushless, false, Set.of(Sensors.ABSOLUTE), Set.of(LogData.POSITION, LogData.VELOCITY, LogData.VOLTAGE, LogData.CURRENT));
    pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    //pivotMotor.setClosedLoopRampRate(0.04);
    //pivotMotor.setOpenLoopRampRate(0.04);
    pivotMotor.setInverted(true);
    pivotMotor.burnFlash();
    
    pivotMotor.setSmartCurrentLimit(25);

    pivotMotorEncoder = new Encoder(kPivot.portA, kPivot.portB);
    pivotMotorEncoder.setReverseDirection(kPivot.invertedEncoder);
    this.resetEncoderCmd();

    pivotFF = new ArmFeedforward(kPivot.kS, kPivot.kG, kPivot.kV, kPivot.kA);

    pivotMotorEncoder.setDistancePerPulse(2 * Math.PI / (kPivot.pulsesPerRevolution * kPivot.gearRatio));
    pivotMotorEncoder.reset();

    profiledPIDController = new ProfiledPIDController(kPivot.kP, kPivot.kI, kPivot.kD, constraints);
    profiledPIDController.reset(getPivotAngle());
    goal = new TrapezoidProfile.State(kPivot.intakeRadiansHome, 0);
    profiledPIDController.setGoal(goal);
    currentSetpoint = profiledPIDController.getSetpoint();
    profiledPIDController.disableContinuousInput(); ///VERY IMPORTANT

    
    // Button to Reset Encoder
    tab.add("Reset Intake Pivot Encoder", resetEncoderCmd());
    tab.addString("Intake Position", () -> goalPosition.name());
  
    tab.addDouble("Intake Pivot Encoder", () -> getPivotAngle());
    //tab.addDouble("", calculateVoltage(getPivotAngle()));
    tab.addDouble("Pivot Motor Current", () -> pivotMotor.getOutputCurrent());
  }

  //MAIN Controls
  public Command setIntakePosition(IntakePosition intakePosition) {
    return this.runOnce(() -> goalPosition = intakePosition)
        .andThen(setIntakePivotPos(intakePosition.angle))
        .onlyWhile(() -> !isAtPosition(goalPosition));
  }

  public Command setIntakePivotPos(double positionRad) {
      return this.run(
              () -> {
                pivotMotor.setVoltage(calculateVoltage(positionRad));
              })
          //.onlyWhile(() -> !isAtPosition(goalPosition));
          //.andThen(() -> isAtPosition = true);      //.isAtPosition(goalPosition))
          .finallyDo(() -> pivotMotor.setVoltage(0)).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
          //.andThen(pivotMotor::disable);
  }

  public Command resetEncoderCmd() {
    return this.runOnce(() -> pivotMotorEncoder.reset()).ignoringDisable(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /*
  public boolean isAtPosition(IntakePosition position) {
    return position.angle == currentSetpoint.position;
  }
  */

  public Command waitUntilIsAtPosition(IntakePosition position) {
    return waitUntil(() -> Math.abs(getPivotAngle() - position.angle) < 0.05);
  }

  public Boolean isAtPosition(IntakePosition position) {
    return Math.abs(getPivotAngle() - position.angle) < 0.05;
  }

  private double calculateVoltage(double angle) {
    // Set appropriate goal
    profiledPIDController.setGoal(angle);

    // Update measurements and get feedback voltage
    double feedbackVoltage = profiledPIDController.calculate(getPivotAngle());

    // Get setpoint from profile
    var nextSetpoint = profiledPIDController.getSetpoint();

    // Calculate acceleration
    var accel = (nextSetpoint.velocity - currentSetpoint.velocity) / 0.02;

    log("Accel", accel);
    log("Next Setpoint velocity", nextSetpoint.velocity);
    log("Current Setpoint velocity", currentSetpoint.velocity);

    double feedForwardVoltage =
    pivotFF.calculate(nextSetpoint.position + kPivot.cogOffset, nextSetpoint.velocity, accel);

    // Log Values
    this.log("FeedbackVoltage", feedbackVoltage);
    this.log("Feedforward voltage", feedForwardVoltage);

    currentSetpoint.position = nextSetpoint.position;
    currentSetpoint.velocity = nextSetpoint.velocity;

    return feedForwardVoltage + feedbackVoltage;
  }

  

  @Log.NT
  public double getPivotAngle() {
    return pivotMotorEncoder.getDistance() + kPivot.encoderOffset;
  }
}
