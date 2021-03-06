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

  private boolean wasInTeleop = true;
  private ElevatorSetpoint setpoint;
  private boolean usingInternal = false;
  private boolean useSetpoints = true;

  public enum ElevatorSetpoint {
    ShipFrontCargo(RobotMap.Values.elevatorFrontShipCargoHeight),
    LowFrontHatch(RobotMap.Values.elevatorFrontBottomHatchHeight),
    MidFrontHatch(RobotMap.Values.elevatorFrontMiddleHatchHeight),
    HighFrontHatch(RobotMap.Values.elevatorFrontTopHatchHeight),
    LowFrontCargo(RobotMap.Values.elevatorFrontBottomCargoHeight),
    MidFrontCargo(RobotMap.Values.elevatorFrontMiddleCargoHeight),
    HighFrontCargo(RobotMap.Values.elevatorFrontTopCargoHeight),
    ShipBackHatch(RobotMap.Values.elevatorBackShipHatchHeight),
    ShipBackCargo(RobotMap.Values.elevatorBackShipCargoHeight),
    LowBackCargo(RobotMap.Values.elevatorBackBottomCargoHeight),
    MidBackCargo(RobotMap.Values.elevatorBackMiddleCargoHeight),
    HighBackCargo(RobotMap.Values.elevatorBackTopCargoHeight),
    Top(RobotMap.Values.elevatorTopHeight),
    Bottom(RobotMap.Values.elevatorBottomHeight);

    private double setpoint;

    private ElevatorSetpoint(double setpoint) {
      setSetpoint(setpoint);
    }

    public double getSetpoint() { return setpoint; }

    public void setSetpoint(double setpoint) {
      this.setpoint = setpoint;
    }

    public static ElevatorSetpoint getCurrent(boolean useInternal) {
      ElevatorSetpoint a = ElevatorSetpoint.Top;

      if (useInternal) {
        a.setSetpoint(Robot.kElevator.getInternalPosition());
      } else {
        a.setSetpoint(Robot.kElevator.getExternalPosition());
      }

      return a;
    }
  }

  public enum POVState {
    FrontCargo(0), BackCargo(270), FrontHatch(90),
    BackHatch(180), Idle(-1);

    private int value;

    private POVState(int value) {
      this.value = value;
    }

    public int getValue() { return value; }

    public static POVState getState(int value) {
      switch (value) {
        case 0:
          return POVState.FrontCargo;
        case 270:
          return POVState.BackCargo;
        case 90:
          return POVState.FrontHatch;
        case 180:
          return POVState.BackHatch;
        default:
          return POVState.Idle;
      }
    }
  }

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
    // Manual Control. Accidently named it teleop but I'm too lazy to change it... WHOOP
    if (!updateSetpoint()) {
      if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.square)) {
        setSpeed(0.8);
        wasInTeleop = true;
      } else if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.triangle)){
        setSpeed(-0.3);
        wasInTeleop = true;
      } else { // Locks elevator when not using it
        if (wasInTeleop) {
          wasInTeleop = false;
          if (usingInternal) {
            setpoint = ElevatorSetpoint.getCurrent(true);
          } else {
            setpoint = ElevatorSetpoint.getCurrent(false);
          }
        }
      }

      if (useSetpoints) { // Moves to new setpoint
        if (usingInternal) {
          setPositionInternal(setpoint.getSetpoint());
        } else {
          setPositionExternal(setpoint.getSetpoint());
        }
      }
    }
  }

  //#region Operator Functions

  public boolean updateSetpoint() {
    POVState state = POVState.getState(Robot.kGamepad2.getPOV(0));

    if (state.equals(POVState.FrontCargo)) {
      if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.x)) {
        setpoint = ElevatorSetpoint.ShipFrontCargo;
      } else if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.square)) {
        setpoint = ElevatorSetpoint.LowFrontCargo;
      } else if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.triangle)) {
        setpoint = ElevatorSetpoint.MidFrontCargo;
      } else if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.circle)) {
        setpoint = ElevatorSetpoint.HighFrontCargo;
      }
    } else if (state.equals(POVState.FrontHatch)) {
      if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.x)) {
        setpoint = ElevatorSetpoint.LowFrontHatch;
      } else if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.square)) {
        setpoint = ElevatorSetpoint.LowFrontHatch;
      } else if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.triangle)) {
        setpoint = ElevatorSetpoint.MidFrontHatch;
      } else if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.circle)) {
        setpoint = ElevatorSetpoint.HighFrontHatch;
      }
    } else if (state.equals(POVState.BackCargo)) {
      if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.x)) {
        setpoint = ElevatorSetpoint.ShipBackCargo;
      } else if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.square)) {
        setpoint = ElevatorSetpoint.LowFrontCargo;
      } else if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.triangle)) {
        setpoint = ElevatorSetpoint.MidFrontCargo;
      } else if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.circle)) {
        setpoint = ElevatorSetpoint.HighFrontCargo;
      }
    } else if (state.equals(POVState.BackHatch)) {
      if (Robot.kGamepad2.getRawButton(RobotMap.PsButtons.x)) {
        setpoint = ElevatorSetpoint.ShipBackHatch;
      }
    } else {
      return false;
    }

    return true;
  }

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
    m_pidController.setReference(p - getInternalPosition(), ControlType.kDutyCycle);
  }

  public void setPositionExternal(double p) {
    m_pidController.setReference(p - getExternalPosition(), ControlType.kDutyCycle);
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
