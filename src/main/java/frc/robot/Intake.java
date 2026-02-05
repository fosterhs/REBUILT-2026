package frc.robot;

public class Intake {
  public enum Mode {HOME, LEFT, RIGHT, STOW} //
  public Mode currMode = Mode.HOME; //
  public double desiredLeftArmPosition = 0.0; //
  public double desiredLeftRollerVelocity = 0.0; //
  public double desiredRightArmPosition = 0.0; //
  public double desiredRightRollerVelocity = 0.0; //

  //
  public Intake() {

  }

  //
  public void init() {

  }

  //
  public void periodic() {
    switch (currMode) {
      case HOME:

      break;

      case LEFT:

      break;

      case RIGHT:

      break;

      case STOW:

      break;
    }
  }

  //
  public void leftIntake() {

  }

  //
  public void rightIntake() {

  }

  //
  public void stowIntake() {

  }

  //
  public Mode getMode() {
    return currMode;
  }

  //
  public double getLeftArmPosition() {
    return 0.0;
  }

  //
  public double getLeftArmDesiredPosition() {
    return 0.0;
  }

  //
  public double getLeftRollerVelocity() {
    return 0.0;
  }

  //
  public double getLeftRollerDesiredVelocity() {
    return 0.0;
  }

  //
  public boolean leftArmInPosition() {
    return true;
  }

  //
  public boolean leftRollerAtSpeed() {
    return true;
  }

  //
  public double getRightArmPosition() {
    return 0.0;
  }

  //
  public double getRightArmDesiredPosition() {
    return 0.0;
  }

  //
  public double getRightRollerVelocity() {
    return 0.0;
  }

  //
  public double getRightRollerDesiredVelocity() {
    return 0.0;
  }
  
  //
  public boolean rightArmInPosition() {
    return true;
  }

  //
  public boolean rightRollerAtSpeed() {
    return true;
  }

  // Returns true or false based on whether the left arm, left roller, right arm, and right roller are all in their correct positions/velocities.
  public boolean isReady() {
    return true;
  }

  //
  public void updateDash() {

  }
}