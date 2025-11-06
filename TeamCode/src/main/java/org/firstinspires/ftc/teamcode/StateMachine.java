package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator;


public class StateMachine {
    public enum State {
        HOME,
        INTAKE_COLOR_CHECK //Color check
    }

    public enum DetectedColor {
        YELLOW,
        RED,
        BLUE,
        NONE
    }

    private ElapsedTime intakeTimer1 = new ElapsedTime();

    private int intakeHomeSubstep = 0;
    private int ColorCheckSubstep = 0;
    public DetectedColor detectedColor = DetectedColor.NONE;  // Variable to track the current color
    private State currentState;
    private final RobotHardware robot;
    private boolean isIntakeRunning = false; // Track if the intake is currently running

    public StateMachine(RobotHardware hardware) {
        this.robot = hardware;
        this.currentState = State.HOME;
    }

    public void setState(State state) {
        System.out.println("Transitioning from " + currentState + " to " + state);
        this.currentState = state;

        //Set all substep trackers to 0
        this.intakeHomeSubstep = 0;

        //Reset all timers
        intakeTimer1.reset();

        updatePositions();
    }

    public void update() { //This is called from the OpMode to make sure long sequences are able to complete
        if (currentState == State.HOME) {
            //robot.disableHoldIntake(); // Disable hold logic when transitioning to HOME
        } else {
            //robot.enableHoldIntake(); // Re-enable hold logic for other states
        }
        //robot.holdIntakeSlidePosition(); // Maintain position when enabled

        switch (currentState) {
            case HOME:
                break;
            case INTAKE_COLOR_CHECK:
                //IntakeColorCheck();
                break;
        }
    }

    public State getState() { //Return the current state
        return currentState;
    }

    private void updatePositions() {
        switch (currentState) {
            case HOME: //Set all positions to home
                robot.rgbIndicator.setColor(rgbIndicator.LEDColors.GREEN);
                //robot.elevatorPivot.setPosition(Constants.elevatorPivotHome);
                //robot.setElevator(Constants.elevatorHome);
                //intakeAllHome();
                break;

            case INTAKE_COLOR_CHECK:
                //IntakeColorCheck();
                break;
        }
    }
}

    /*
    private void IntakeColorCheck(){
        System.out.println("Color Check Substep:" + ColorCheckSubstep);
        switch (ColorCheckSubstep){
            case 0:  // Intake Until Button Pressed or Timeout
                if (!isIntakeTimerReset) {
                    IntakeTimer.reset();  // Reset the timer only once upon entering case 0
                    isIntakeTimerReset = true;
                }
                if (robot.intakeTouch.getState() && robot.intakeDistance.getDistance(DistanceUnit.MM) > Constants.intakeProx && IntakeTimer.seconds() < 5) {
                    if (robot.allianceColorRed){
                        robot.rgbIndicator.setColor(rgbIndicator.LEDColors.RED); // Set LED while searching
                    } else {
                        robot.rgbIndicator.setColor(rgbIndicator.LEDColors.BLUE);
                    }
                    //robot.runIntake(RobotHardware.ActiveIntake.IN); //Run intake

                } else {
                    robot.runIntake(RobotHardware.ActiveIntake.STOP); // Stop intake
                    isIntakeRunning = false;
                    if (!robot.intakeTouch.getState() || robot.intakeDistance.getDistance(DistanceUnit.MM) < Constants.intakeProx) {
                        System.out.println("Limit switch triggered, moving to next step.");
                    } else {
                        System.out.println("Timeout reached, moving to next step.");
                    }
                    ColorCheckSubstep++;
                    isIntakeTimerReset = false;  // Reset the flag for the next transition
                    break;
                }
                break;
            case 1: //Check the color and set the LED
                System.out.println(String.format("Intake RGB: %d, %d, %d", robot.intakeColor.red(), robot.intakeColor.green(), robot.intakeColor.blue()));

                if (robot.intakeColor.red() > Constants.intakeColorRed && robot.intakeColor.green() > Constants.intakeColorGreen){
                    //YELLOW
                    detectedColor = DetectedColor.YELLOW;
                    robot.rgbIndicator.setColor(rgbIndicator.LEDColors.YELLOW);
                    ColorCheckSubstep++;
                    break;
                } else if (robot.intakeColor.red() > Constants.intakeColorRed) {
                    //RED
                    detectedColor = DetectedColor.RED;
                    robot.rgbIndicator.setColor(rgbIndicator.LEDColors.RED);
                    ColorCheckSubstep++;
                    break;
                } else if (robot.intakeColor.blue() > Constants.intakeColorBlue) {
                    //BLUE
                    detectedColor = DetectedColor.BLUE;
                    robot.rgbIndicator.setColor(rgbIndicator.LEDColors.BLUE);
                    ColorCheckSubstep++;
                    break;
                } /*else { //TODO add a timeout to this
                    //Not Detected - this probably shouldn't happen?
                    detectedColor = DetectedColor.NONE;
                    robot.rgbIndicator.setColor(rgbIndicator.LEDColors.OFF);
                    ColorCheckSubstep++;
                    break;
                }

            case 2: //Do we actually want this color?
                System.out.println("Detected Color: " + detectedColor);
                if (!isEjectTimerReset){
                    ColorCheckEjectTimer.reset();
                    isEjectTimerReset = true;
                }

                if (detectedColor == DetectedColor.YELLOW ||
                        (detectedColor == DetectedColor.RED && robot.allianceColorRed) ||
                        (detectedColor == DetectedColor.BLUE && robot.allianceColorBlue)) {
                    ColorCheckSubstep++;
                    break;
                } else {
                    //Got a color we don't want, spit it out and restart this case
                    robot.runIntake(RobotHardware.ActiveIntake.OUT);
                    if (ColorCheckEjectTimer.seconds() > 0.50){
                        robot.runIntake(RobotHardware.ActiveIntake.STOP);
                        detectedColor = DetectedColor.NONE;
                        isEjectTimerReset = false;
                        isIntakeTimerReset = false;  // Ensure timer resets when restarting case 0
                        ColorCheckSubstep = 0;
                        break;
                    }
                    break;
                }
            case 3:
                ColorCheckSubstep = 0;
                detectedColor = DetectedColor.NONE; //reset this for next cycle
                setState(State.PICKUP_RETRACT);
                break;
        }
    }
    */
