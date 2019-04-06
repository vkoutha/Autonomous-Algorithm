package org.usfirst.frc.team11.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;

public class Movements {
	
	private Timer timer = new Timer();
	private TalonSRX lTal;
	private TalonSRX rTal;
	private TalonSRX armTal;
	private TalonSRX intakeTal;

	//Counts per inches of drive train encoders
	private double CPI;

	//Used to compensate for the extra distance travelled when moving at high speeds (change/remove if necessary)
	private double speedCompensator;

	//ArrayList of booleans used to monitor whether or not each step of autonomous has been finished or not (true = finished, false = incomplete)
	private ArrayList<Boolean> isFinished = new ArrayList<Boolean>();

	//Used for simultaneous actions with end effector components to monitor whether or not an end effector action has been completed
	private boolean endEffectorFinished[] = {false, false}; //0 = Arm, 1 = Intake, add more if needed
	
	/**
	 * @param lTal 					Represents Left Talon Motor of drive train controlling the left drive
	 * @param rTal 					Represents Right Talon Motor of drive train controlling the right drive
	 * @param armTal 				Represents Arm Talon Motor of robot controlling the arm
	 * @param intakeTal 			Represents the Intake Talon Motor controlling the intake
	 * @param CPR	 				Represents counts per revolutions of the drive train encoders
	 * @param wheelCircumference 	Represents the circumference of the drive train wheels (diameter * 3.14)
	 * 
	 * Other motors/speed controllers can be added based on need
	 */ 
	public Movements(TalonSRX lTal, TalonSRX rTal, TalonSRX armTal, TalonSRX intakeTal, double CPR, double wheelCircumference){
		this.lTal = lTal;
		this.rTal = rTal;
		this.armTal = armTal;
		this.intakeTal = intakeTal;
		//CPI represents counts per inch of encoder
		CPI = CPR/wheelCircumference;
		//Adds a default value to the isFinished arraylist to automatically start the first step of autonomous
		isFinished.add(0, true);
	}

	/**
	 * @param distance 			Represents distance needed to be travelled to in inches
	 * @param speed 			Represents the desired speed to be travelled at (positive, 0-1)
	 * @param numInSequence 	Represents the order in which the action occurs in, in autonomous (i.e. 1 if the first action, 2 if second action, etc.)
	 * @param simultaneously 	Represents and end effector action will be running at the same time with it (true if simultaneous, false if not)
	 */
	public void driveForward(double distance, double speed, int numInSequence){
		//Compensates for extra distance by multiplying speed by 10 and removing that from target distance
		speedCompensator = speed * 15;
		//If nothing will happen along with this method
		/**
		* If talon's encoder counts are less than or equal to the encoder counts for the required distance, and the previous action in autonomous
		* has been completed
		*/
		if(rTal.getSelectedSensorPosition(0) <= (distance-speedCompensator) * CPI && isFinished.get(numInSequence-1) == true){
			//If the action's corresponding boolean has not yet been added to the ArrayList
			if(isFinished.size() == numInSequence){
			//Adding the action's corresponding boolean to the ArrayList
			isFinished.add(numInSequence, false);
			}
			//Setting speed for talon motors
			rTal.set(ControlMode.PercentOutput, speed * -1);
			lTal.set(ControlMode.PercentOutput, speed);
		/**
		* If the encoder counts are now greater than the necessary encoder counts for the required distance and the action's corresponding boolean
		*has already been added to the ArrayList
		*/
		}else if (isFinished.size() == numInSequence+1){
			//Set the previous action's boolean to false so that the current action does not run again
			isFinished.set(numInSequence-1, false);
			//Set the current action to true to start the next autonomous action
			isFinished.set(numInSequence, true);
			//Reset encoder positions for the next autonomous action
			rTal.setSelectedSensorPosition(0, 0, 0);
			lTal.setSelectedSensorPosition(0, 0, 0);
		}
	}
	
	//Method used to drive backwards, all comments from previous method apply here as well
	public void driveBackward(double distance, double speed, int numInSequence){
		speedCompensator = speed * 15;
			if(rTal.getSelectedSensorPosition(0) * -1 <= (distance-speedCompensator) * CPI && isFinished.get(numInSequence-1) == true){
				if(isFinished.size() == numInSequence){
					isFinished.add(numInSequence, false);
					}
				rTal.set(ControlMode.PercentOutput, speed);
				lTal.set(ControlMode.PercentOutput, speed * -1);
			}else if (isFinished.size() == numInSequence+1){
				isFinished.set(numInSequence, true);
				isFinished.set(numInSequence-1, false);
				rTal.setSelectedSensorPosition(0, 0, 0);
				lTal.setSelectedSensorPosition(0, 0, 0);
			}
	}
	
	//Method used to turn left, all comments from previous method apply here as well
	public void turnLeft(double distance, double speed, int numInSequence){
		speedCompensator = speed * 15;
			//Monitor the right talon for ease of turning (right wheels move forward while left wheels move backward when you turn left)
			if(rTal.getSelectedSensorPosition(0) <= (distance-speedCompensator) * CPI && isFinished.get(numInSequence-1) == true){
				if(isFinished.size() == numInSequence){
					isFinished.add(numInSequence, false);
					}
				rTal.set(ControlMode.PercentOutput, speed);
				lTal.set(ControlMode.PercentOutput, speed);
			}else if (isFinished.size() == numInSequence+1){
				isFinished.set(numInSequence, true);
				isFinished.set(numInSequence-1, false);
				rTal.setSelectedSensorPosition(0, 0, 0);
				lTal.setSelectedSensorPosition(0, 0, 0);
			}
	}
	
	public void turnRight(double distance, double speed, int numInSequence){
		speedCompensator = speed * 15;
			//Monitor the left talon for ease of turning (left wheels move forward while right wheels move backward when you turn right)
			if(lTal.getSelectedSensorPosition(0) <= (distance-speedCompensator) * CPI && isFinished.get(numInSequence-1) == true){
				if(isFinished.size() == numInSequence){
					isFinished.add(numInSequence, false);
					}
				rTal.set(ControlMode.PercentOutput, speed * -1);
				lTal.set(ControlMode.PercentOutput, speed * -1);
			}else if (isFinished.size() == numInSequence+1){
				isFinished.set(numInSequence, true);
				isFinished.set(numInSequence-1, false);
				rTal.setSelectedSensorPosition(0, 0, 0);
				lTal.setSelectedSensorPosition(0, 0, 0);
			}
	}
	
	//Method used to stop drive train only
	public void stopDriveTrain(int numInSequence){
		if(isFinished.get(numInSequence-1) == true){
			if(isFinished.size() == numInSequence){
				isFinished.add(numInSequence, false);
			}
			rTal.set(ControlMode.PercentOutput, 0);
			lTal.set(ControlMode.PercentOutput, 0);
		}else if (isFinished.size() == numInSequence+1){
			isFinished.set(numInSequence, true);
			isFinished.set(numInSequence-1, false);
			rTal.setSelectedSensorPosition(0, 0, 0);
			lTal.setSelectedSensorPosition(0, 0, 0);
		}
	}
	
	public void wait(double time, int numInSequence){
		if(timer.get() <= time && isFinished.get(numInSequence-1) == true){
			if(isFinished.size() == numInSequence){
				isFinished.add(numInSequence, false);
			}
			if(timer.get() == 0){
				timer.start();
			}
		}else if (isFinished.size() == numInSequence+1){
			isFinished.set(numInSequence, true);
			isFinished.set(numInSequence-1, false);
			timer.stop();
			timer.reset();
		}
	}
	
	/**
	 * @param time  				Represents the amount of time to move the arm for (can be changed to encoder counts if necessary)
	 * @param velocity 				Represents the target velocity to move at for the arm (can be changed to PercentOutput control mode if necessary)
	 * @param numInSequence 		Represents the order in which the action occurs in, in autonomous (i.e. 1 if the first action, 2 if second action, etc.)
	 * @param simultaneously 		Represents and end effector action will be running at the same time with it (true if simultaneous, false if not)
	 */
	public void moveArm(double time, double velocity, int numInSequence, boolean simultaneously){
		//If occurring independently, without any simultaneous drive train movement
		if(simultaneously == false){
			//If the current time of the timer is less than the desired time to run the intake for and the previous action in autonomous has been completed
			if(timer.get() <= time && isFinished.get(numInSequence-1) == true){
				//If the timer is equal to 0 (has not been started)
				if(timer.get() == 0){
					//Start the timer
					timer.start();
				}
				//If the action has not been yet added to the ArrayList
				if(isFinished.size() == numInSequence){
					//Add the action to the ArrayList
					isFinished.add(numInSequence, false);
				}
				//Set the velocity of the arm to the desired velocity
				armTal.set(ControlMode.Velocity, velocity);
			//If the time to raise the arm for has been crossed and the action is in the ArrayList
			}else if (isFinished.size() == numInSequence+1){
				//Set the current action's corresponding boolean to true to start the next autonomous action
				isFinished.set(numInSequence, true);
				//Set the previous action's corresponding boolean to false so that the current action does not repeat again
				isFinished.set(numInSequence-1, false);
				//Stop the arm from moving (can be changed to a higher/lower number if the arm does not remain at a constant position)
				armTal.set(ControlMode.Velocity, 0);
				//Stop the timer
				timer.stop();
				//Reset the timer to 0
				timer.reset();
				
			}
		
		//If another drive train action is occurring with this end effector action
		}else if (simultaneously == true){
			//If the action has not yet been added to the ArrayList
			if(isFinished.size() == numInSequence){
				//Add the action to the ArrayList
				isFinished.add(numInSequence, false);
				//Set endEffectorFinished to false in case it has a value of true from another previous use
				endEffectorFinished[0] = false;
				}
			//If the timer is less than the desired time, the previous autonomous action is completed, and the action has not been completed before
			if(timer.get() <= time && isFinished.get(numInSequence-1) == true && endEffectorFinished[0] == false){
				//If the timer is equal to 0
				if(timer.get() == 0){
					//Start the timer
					timer.start();
				}
				//Set the arm to the desired velocity
				armTal.set(ControlMode.Velocity, velocity);
			//If the timer has passed the desired time and the action is in the ArrayList
			}else if (isFinished.size() == numInSequence+1){
				//Set the endEffectorFinished boolean to true as the action has been completed
				endEffectorFinished[0] = true;
				//Stop the arm from moving (can be changed to a higher/lower velocity if arm does not remain at constant position)
				armTal.set(ControlMode.Velocity, 0);
			}
		}
	}
	
	
	/**
	 * @param time  				Represents the amount of time to use the intake for (can be changed to encoder counts if necessary)
	 * @param velocity 				Represents the target speed to spin the intake at
	 * @param numInSequence 		Represents the order in which the action occurs in, in autonomous (i.e. 1 if the first action, 2 if second action, etc.)
	 * @param simultaneously 		Represents and end effector action will be running at the same time with it (true if simultaneous, false if not)
	 * See previous end effector method comments
	 */
	public void useIntake(double time, double speed, int numInSequence, boolean simultaneously){
		if(simultaneously == false){
			if(timer.get() <= time && isFinished.get(numInSequence-1) == true){
				if(timer.get() == 0){
					timer.start();
				}
				if(isFinished.size() == numInSequence){
				isFinished.add(numInSequence, false);
				}
				intakeTal.set(ControlMode.PercentOutput, speed);
			}else if (isFinished.size() == numInSequence+1){
				isFinished.set(numInSequence, true);
				isFinished.set(numInSequence-1, false);
				timer.stop();
				timer.reset();
			}
		}else if (simultaneously == true){
			if(isFinished.size() == numInSequence){
				isFinished.add(numInSequence, false);
				endEffectorFinished[1] = false;
				}
			if(timer.get() <= time && isFinished.get(numInSequence-1) == true && endEffectorFinished[1] == false){
				if(timer.get() == 0){
					timer.start();
				}
				intakeTal.set(ControlMode.PercentOutput, speed);
			}else if (isFinished.size() == numInSequence+1){
				endEffectorFinished[1] = true;
				timer.stop();
				timer.reset();
			}
		}
	}

}
