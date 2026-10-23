package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public enum ElevatorState {
    //RIGHT AND LEFT: determined by straight forward (0 on gyro/starting position)
  START(0, 0), //Default position for elevator, with elevator down and arm straight up
  INTAKE_RIGHT(Math.PI/2 , pickUpStandingHeight), //Picks up standing cones on the right side
  INTAKE_LEFT(-Math.PI/2, pickUpStandingHeight),  //Picks up standing cones on the left side
  INTAKE_TOPPLE_RIGHT(toppledAngle, pickUpToppledHeight, true), //Picks up toppled cones on the right side
  INTAKE_TOPPLE_LEFT(-toppledAngle, pickUpToppledHeight, true), //Picks up toppled cones on the left side
  SCORE_RIGHT(0, scoreLowHeight), //Score cones (lower-not max height) on the right side
  SCORE_LEFT(0, scoreLowHeight),  //Score cones (lower-not max height) on the left side
  SCORE_MAX_RIGHT(Math.PI/4, scoreHighHeight), //Scores cones (max height of elevator) on the right side
  SCORE_MAX_LEFT(-Math.PI/4, scoreHighHeight); //Scores cones (max height of elevator) on the left side

  private double armAngle; 
  private double height;
  private boolean deployBar = false;

  /**
 * Constructs a position for the elevator with both a height and arm angle
 * 
 * @param armAngle The angle of the arm in Radians, with 0 being straight up and positive being clockwise 
 * (while robot facing straight forwards)
 * @param height The height of the elevator in Meters with 0 being at the bottom
 * @throws IllegalArgumentException If the height is less than zero.
 */
  private ElevatorState(double armAngle, double height) throws IllegalArgumentException{
    if (height < 0) {
        throw new IllegalArgumentException("Height cannot be negative.");
    } else {
        this.armAngle = armAngle;
        this.height = height;
    }
  }

  /**
 * Constructs a position for the elevator with both a height and arm angle
 * 
 * @param armAngle The angle of the arm in Radians, with 0 being straight up and positive being clockwise 
 * (while robot facing straight forwards)
 * @param height The height of the elevator in Meters with 0 being at the bottom
 * @param deployBar defaults to false, true for tipped cone -> it will flip it back up during intake process
 * @throws IllegalArgumentException If the height is less than zero.
 */
  private ElevatorState(double armAngle, double height, boolean deployBar) {
    this.armAngle = armAngle;
    this.height = height;
    this.deployBar = deployBar;
  }

  /**
   * @return height of elevator in meters, with 0 being bottomed out
   */
  public double getHeight(){
    return this.height;
  }

  /**
   * @return angle of the elevator arm in radians, with 0 being straight up and positive is
   *     clockwise
   */
  public double getAngle(){
    return this.armAngle;
  }

  /**
   * @return whether during the intake process robot will deploy the bar to flip cones back up
   */
  public boolean shouldDeployBar(){
    return this.deployBar;
  }
}