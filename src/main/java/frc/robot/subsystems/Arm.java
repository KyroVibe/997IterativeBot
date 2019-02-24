package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Arm {

  private CANSparkMax m_sparkMax;
  private CANEncoder m_encoder;
  private CANPIDController m_pidController;
  private CANDigitalInput m_frontLimitSwitch, m_backLimitSwitch;
  private CANifier m_canifier;
  private Solenoid m_discBrake;

  private double ramp = 1.66;

  //#region Teleop Data

  private final double MAX = 1022;
  private final double LIMIT = 500;
  private double prevRead = -1;
  private int revs = 0;
  private int flipModifier = 1;

  //#endregion

  public Arm() {
    m_sparkMax = new CANSparkMax(RobotMap.Ports.armSpark, MotorType.kBrushless);
    m_sparkMax.setInverted(true);

    m_encoder = m_sparkMax.getEncoder();
    m_encoder.setPosition(0);
    m_encoder.setPositionConversionFactor(42);

    m_pidController = m_sparkMax.getPIDController();
    m_pidController.setP(0.002);
    m_pidController.setI(0);
    m_pidController.setD(0);
    m_pidController.setFF(0);
    m_pidController.setOutputRange(-0.6, 0.6);

    m_frontLimitSwitch = new CANDigitalInput(m_sparkMax, LimitSwitch.kForward, LimitSwitchPolarity.kNormallyOpen);
    m_frontLimitSwitch.enableLimitSwitch(true);
    m_backLimitSwitch = new CANDigitalInput(m_sparkMax, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyOpen);
    m_backLimitSwitch.enableLimitSwitch(true);

    m_canifier = new CANifier(RobotMap.Ports.armCANifier);

    m_discBrake = new Solenoid(RobotMap.Ports.discBrakeSolenoid);
  }

  public void teleopPeriodic() {

  }

  //#region Operator Functions

  public void setSpeed(double s) {
    m_sparkMax.set(s);
  }

  public void setRampSpeed(double s) {
    double prev = m_sparkMax.get();
    double newS = s;
    double maxIncrement = Robot.kDeltaTime * ramp;

    if (Math.abs(s - prev) > maxIncrement) {
      double sign = (s - prev) / Math.abs(s - prev);
      newS = (maxIncrement * sign) + prev;
    }

    setSpeed(newS);
  }

  public void setInternalPosition(double p) {
    m_pidController.setReference(p - getInternalPosition(), ControlType.kDutyCycle);
  }

  public void setExternalPosition(double p) {
    m_pidController.setReference(p - getExternalPosition(), ControlType.kDutyCycle);
  }

  public boolean toggleDiscBrake() {
    m_discBrake.set(!m_discBrake.get());
    return m_discBrake.get();
  }

  public void setDiscBrake(boolean s) {
    m_discBrake.set(s);
  }

  //#endregion

  //#region Utility Functions

  public double getExternalPosition() {
    double newVal = getRawEncoder();

    if (prevRead == -1) { 
      prevRead = newVal;
    }

    if (Math.abs(prevRead - newVal) > LIMIT) {
      if (newVal > prevRead) {
        revs -= flipModifier;
      } else {
        revs += flipModifier;
      }
    }
    prevRead = newVal;
    return (int)(((revs * MAX) + (newVal/* - initRead*/)));
  }

  public double getRawEncoder() {
    double[] a = new double[2];
    m_canifier.getPWMInput(PWMChannel.PWMChannel0, a);
    SmartDashboard.putNumber("Duty Cycle", a[1]);
    return a[0];
  }

  public double getInternalPosition() {
    return m_encoder.getPosition();
  }

  //#endregion

}