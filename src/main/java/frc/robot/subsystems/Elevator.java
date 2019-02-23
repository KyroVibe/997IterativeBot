/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Elevator {

  public double kP = 0.00003, kI = 0, kD = 0, kFF = 0;

  private CANSparkMax m_master, m_follower;
  private CANEncoder m_encoder;

  private CANPIDController m_pidController;
  private CANDigitalInput m_limitSwitchTop;
  private CANDigitalInput m_limitSwitchBottom;
  private CANifier m_canifier;

  private double ramp = 1.66;

  //#region Teleop Data

  private boolean wasInTeleop = false;
  private double setpoint = 0;
  private boolean usingInternal = false;
  private boolean useSetpoints = true;

  //#endregion

  public Elevator() {
    m_master = new CANSparkMax(RobotMap.Ports.masterSpark, MotorType.kBrushless);
    m_follower = new CANSparkMax(RobotMap.Ports.followerSpark, MotorType.kBrushless);

    m_master.setInverted(true);
    m_follower.setInverted(true);

    m_follower.follow(m_master, true);

    m_encoder = m_master.getEncoder();
    m_encoder.setPositionConversionFactor(42);

    m_limitSwitchTop = new CANDigitalInput(m_master, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyOpen);
    m_limitSwitchTop.enableLimitSwitch(true);
    
    m_limitSwitchBottom= new CANDigitalInput(m_master, LimitSwitch.kForward , LimitSwitchPolarity.kNormallyOpen);
    m_limitSwitchBottom.enableLimitSwitch(true);

    m_pidController = m_master.getPIDController();
    m_pidController.setOutputRange(-0.7, 0.7);
    m_pidController.setP(0.2);
    m_pidController.setI(0.0);
    m_pidController.setD(0.0);
    m_pidController.setFF(0.0);

    SmartDashboard.putNumber("Elevator Pid P", kP);
    SmartDashboard.putNumber("Elevator Pid I", kI);
    SmartDashboard.putNumber("Elevator Pid D", kD);
    SmartDashboard.putNumber("Elevator Pid FF", kFF);

    m_canifier = new CANifier(RobotMap.Ports.elevatorCANifier);
    m_canifier.setQuadraturePosition(0, 5);
  }

  public void teleopPeriodic() {
    if (Robot.kGamepad2.getRawButton(RobotMap.Buttons.buttonX)) {
      setSpeed(0.8);
      wasInTeleop = true;
    } else if (Robot.kGamepad2.getRawButton(RobotMap.Buttons.buttonY)){
      setSpeed(-0.3);
      wasInTeleop = true;
    } else {
      if (wasInTeleop) {
        wasInTeleop = false;
        if (usingInternal) {
          setpoint = getInternalPosition();
        } else {
          setpoint = getExternalPosition();
        }
      }

      if (useSetpoints) {
        if (usingInternal) {
          setPositionInternal(setpoint);
        } else {
          setPositionExternal(setpoint);
        }
      }
    }
  }

  //#region Operator Functions

  public void setSpeed(double s) {
    m_master.set(s);
  }

  public void setRampSpeed(double s) {
    double prev = m_master.get();
    double newS = s;
    boolean didMod = false;
    double maxIncrement = Robot.kDeltaTime * ramp;

    if (Math.abs(s - prev) > maxIncrement) {
      double sign = (s - prev) / Math.abs(s - prev);
      newS = (maxIncrement * sign) + prev;
    }

    if (didMod) { // Did you mod it?
      if (newS < 0) { // Is the new values moving up
        if ((getExternalPosition() < RobotMap.Values.bottomElevatorAccelPosLimit) && (newS < RobotMap.Values.bottomElevatorLimitVelocity)) { // Is it approching the bottom of the elevator and is going rather fast?
          newS = RobotMap.Values.bottomElevatorLimitVelocity; // Limit the velocity even more
        }
      } else if ((getExternalPosition() > RobotMap.Values.topElevatorAccelPosLimit) && (newS > RobotMap.Values.topElevatorLimitVelocity)) { // Is it approching the top of the elevator and is going rather fast?
        newS = RobotMap.Values.topElevatorLimitVelocity; // Limit the velocity even more
      }
    }

    setSpeed(newS);
  }

  public void setPositionInternal(double p) {
    m_pidController.setReference(p - getInternalPosition(), ControlType.kPosition);
  }

  public void setPositionExternal(double p) {
    m_pidController.setReference(p - getExternalPosition(), ControlType.kPosition);
  }

  //#endregion

  //#region Utility Functions

  public double getExternalPosition() {
    return m_canifier.getQuadraturePosition();
  }

  public double getInternalPosition() {
    return m_encoder.getPosition();
  }

  //#endregion

}
