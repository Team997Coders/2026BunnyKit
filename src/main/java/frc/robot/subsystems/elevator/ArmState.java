import static frc.robot.subsystems.elevator.ElevatorConstants.*;
public enum ArmState {
  START(0, startHeight, false), 
  PICKUPSTANDINGRIGHT(Math.PI/2 , pickUpStandingHeight, false), 
  PICKUPSTANDINGLEFT(-Math.PI/2, pickUpStandingHeight, false), 
  PICKUPTOPPLEDRIGHT(toppledAngle, pickUpToppledHeight, true), 
  PICKUPTOPPLEDLEFT(-toppledAngle, pickUpToppledHeight, true), 
  SCORELOWRIGHT(0, scoreLowHeight, false), 
  SCORELOWLEFT(0, scoreLowHeight, false), 
  SCOREHIGHRIGHT(Math.PI/4, scoreHighHeight, false), 
  SCOREHIGHLEFT(-Math.PI/4, scoreHighHeight, false);

  double armPos;
  double elevatorPos;
  boolean intake;

  private ArmState(double armPos, double elevatorPos, boolean intake) {
    this.armPos = armPos;
    this.elevatorPos = elevatorPos;
    this.intake = intake;
  }

  public double getHeight(){
    return this.elevatorPos;
  }

  public double getAngle(){
    return this.armPos;
  }
}