package frc.robot;

public class RobotMap {

  public class Ports {

    public static final int
      leftTalon = 4, rightTalon = 1, leftVictor1 = 5,
      leftVictor2 = 6, rightVictor1 = 2, rightVictor2 = 3,
      liftGearSolenoid = 1, bellyPanInfared = 1, masterSpark = 42,
      followerSpark = 43, armSpark = 53, elevatorCANifier = 23,
      armCANifier = 29;

  }

  public class Buttons {

    public static final int
      buttonA = 1, buttonB = 2, buttonX = 3,
      buttonY = 4, buttonLeftBumper = 5, buttonRightBumper = 6; // left and right might be reversed

  }

  public class Values {

    public static final double
      elevatorBackTopHatchHeight = 0, //impossible
      elevatorBackMiddleHatchHeight = 0, //impossible
      elevatorBackBottomHatchHeight = 0, //impossible
      elevatorBackShipHatchHeight = 8900, //impossible

      elevatorBackTopCargoHeight = 52400, // probably higher
      elevatorBackMiddleCargoHeight = 34400,
      elevatorBackBottomCargoHeight = 9100,
      elevatorBackShipCargoHeight = 24650,

      elevatorFrontTopHatchHeight = 52400, // probably higher
      elevatorFrontMiddleHatchHeight = 30530,
      elevatorFrontBottomHatchHeight = 1520,
      elevatorFrontShipHatchHeight = 1520,

      elevatorFrontTopCargoHeight = 52400,
      elevatorFrontMiddleCargoHeight = 45360,
      elevatorFrontBottomCargoHeight = 22200,
      elevatorFrontShipCargoHeight = 38125,

      elevatorCollectCargoHeight = 0,
    
      elevatorSafeFlipHeight = 23000,
    
      armBackParallel = 660,
      armFrontParallel = 150,
      armVertical = 415,

      bottomElevatorAccelPosLimit = 5000,
      bottomElevatorLimitVelocity = -0.15,
      topElevatorAccelPosLimit = 49000,
      topElevatorLimitVelocity = 0.4;

  }

}