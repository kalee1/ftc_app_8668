package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Tier 2 extends <code>Error404_Hardware_Tier1</code> class and covers additional driving and servo methods.
 *
 * @author Team 8668
 * @see Error404_Hardware_Tier1
 */
public class Error404_Hardware_Tier2 extends Error404_Hardware_Tier1 { //VERSION 2.1.2

    protected MiniPID turnControl;
    protected MiniPID driveControl;

    /** When the driver hits start sets up PID control. */
    @Override public void start()
    {
        turnControl = new MiniPID( .01, 0, .018 );
        turnControl.setOutputLimits(1.0);
        turnControl.setOutputMin( 0.25 );
        turnControl.reset();

        driveControl = new MiniPID( .025, 0, 0 );
        driveControl.setOutputLimits(1.0);
        driveControl.setOutputMin( 0.25 );
        driveControl.reset();

        super.start();
    }

    /** this method is used to stop all the drive motors. */
    public void chassis_stopEverything(){
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
        turnControl.reset();
        driveControl.reset();
    }

    /**
     * THe intake_glyphIntake method triggers the two intake motors that will suck in or spit out a glyph.
     *
     * @param inOrOut  A string that determines the motor powers and direction. */
    public void intake_glyphIntake(String inOrOut){
        if(inOrOut.toLowerCase().equals("outSlow")){
            rightGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightGlyph.setDirection(FORWARD);
            leftGlyph.setDirection(FORWARD);
            leftGlyph.setPower(0.2);
            rightGlyph.setPower(0.2);
        }
        if(inOrOut.toLowerCase().equals("out")){
            rightGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightGlyph.setDirection(FORWARD);
            leftGlyph.setDirection(FORWARD);
            leftGlyph.setPower(0.5);
            rightGlyph.setPower(0.5);
        }
        if(inOrOut.toLowerCase().equals("stop")){
            rightGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightGlyph.setDirection(FORWARD);
            leftGlyph.setDirection(REVERSE);
            leftGlyph.setPower(0);
            rightGlyph.setPower(0);
        }
        if(inOrOut.toLowerCase().equals("in")){
            rightGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightGlyph.setDirection(REVERSE);
            leftGlyph.setDirection(REVERSE);
            leftGlyph.setPower(0.5);
            rightGlyph.setPower(0.5);
        }
    }

    /**
     * Combines direction and power into one drive method that drives forward and backward.
     *
     * @param power  determines the power which the motors run at*/
    public void chassis_driveStraightCombo(double power){
        if(power>0){
            chassis_driveStraight("RUE",power,"f",0);
        }
        else if(power<0){
            chassis_driveStraight("RUE",(power*(-1)),"r",0);
        }
        else if(power==0){
            chassis_stopEverything();
        }
    }

    /**
     * Combines direction and power into one drive method that slides left and right.
     *
     * @param power  determines the power which the motors run at */
    public void chassis_slideSidewaysCombo(double power){
        if(power>0){
            chassis_slide_sideways("RUE",power,"r",0);
        }
        else if(power<0){
            chassis_slide_sideways("RUE",(power*(-1)),"l",0);
        }
        else if(power==0){
            chassis_stopEverything();
        }
    }

    /**
     * Combines direction and power into one turning method that spins the robot left and right.
     *
     * @param power  determines the power which the motors run at */
    public void chassis_pointTurnCombo(double power){
        if(power>0){
            chassis_pointTurn("RUE",power,"r",0);
        }
        else if(power<0){
            chassis_pointTurn("RUE",(power*(-1)),"l",0);
        }
        else if(power==0){
            chassis_stopEverything();
        }
    }

    /**
     * Uses PID control to turn to a particular heading.
     *
     * @param targetHeading specifies the direction the robot needs to face
     */
    public boolean chassis_pointTurnGyro(double targetHeading, boolean extendedGyro )
    {
        boolean done = false;
        double currentHeading = chassis_getHeadingDbl();
        if ( extendedGyro )
        {
            if ( currentHeading < 0.0 )
            {
                currentHeading = 360.0 + currentHeading;
            }
        }
        double motor_power = 0;

        if ( Math.abs( targetHeading ) > 180.0 )
        {
            // Reject any targetHeadings outside of the allowable rage of the gyro (+/- 180 degrees)
            done = true;
        }
        else
        {
            motor_power = turnControl.getOutput( currentHeading, targetHeading );
            chassis_pointTurnCombo( -1*motor_power );

            if ( Math.abs(currentHeading - targetHeading) < 4.0 )
            {
                done = true;
            }
        }

        return done;
    }

    /**
     * Uses several parameters to set the motors the the directions needed for more complicated maneuvers.
     *
     * @param movement  specifies type of movement (i.e., straight, turn, slide)
     * @param direction  sets motor directions for the encoders */
    public void chassis_setMultipleDirections(String movement, String direction){
        if(movement.equals("straight")){
            if(direction.equals("forward")){
                chassis_set_direction(leftFront, "f");
                chassis_set_direction(leftRear, "r");
                chassis_set_direction(rightFront, "r");
                chassis_set_direction(rightRear, "r");
            }
            if(direction.equals("reverse")){
                chassis_set_direction(leftFront, "r");
                chassis_set_direction(leftRear, "f");
                chassis_set_direction(rightFront, "f");
                chassis_set_direction(rightRear, "f");
            }
        }
        if(movement.equals("turn")){
            if(direction.equals("right")){
                chassis_set_direction(leftFront, "f");
                chassis_set_direction(leftRear, "r");
                chassis_set_direction(rightFront, "f");
                chassis_set_direction(rightRear, "f");
            }
            if(direction.equals("left")){
                chassis_set_direction(leftFront, "r");
                chassis_set_direction(leftRear, "f");
                chassis_set_direction(rightFront, "r");
                chassis_set_direction(rightRear, "r");
            }
        }
        if(movement.equals("slide")){
            if(direction.equals("right")){
                chassis_set_direction(leftFront, "f");
                chassis_set_direction(rightRear, "r");
                chassis_set_direction(rightFront, "f");
                chassis_set_direction(leftRear, "f");
            }
            if(direction.equals("left")){
                chassis_set_direction(leftFront, "r");
                chassis_set_direction(rightRear, "f");
                chassis_set_direction(rightFront, "r");
                chassis_set_direction(leftRear, "r");
            }
        }
    }


    /**
     * Uses various parameters to make the robot drive straight
     *
     * @param mode  a String that gives the motor mode
     * @param power  a Double that gives the power
     * @param direction  a String that tells the direction (forward "f" or reverse "r")
     * @param position  an Int that tells the motor position on the robot
     */
    //tells robot to drive straight at a certain power in a direction until the desired position is reached
    public void chassis_driveStraight(String mode, double power, String direction, int position) {
        position= chassis_distance2encoder(position,6,1);
        if (direction.toLowerCase().equals("f")) {
            chassis_set_direction(leftFront, "f");
            chassis_set_direction(leftRear, "r");
            chassis_set_direction(rightFront, "r");
            chassis_set_direction(rightRear, "r");
        } else {
            chassis_set_direction(leftFront, "r");
            chassis_set_direction(leftRear, "f");
            chassis_set_direction(rightFront, "f");
            chassis_set_direction(rightRear, "f");
        }
        chassis_set_mode(leftFront, mode);
        chassis_set_mode(leftRear, mode);
        chassis_set_mode(rightFront, mode);
        chassis_set_mode(rightRear, mode);
        chassis_set_position(leftFront, position);
        chassis_set_position(leftRear,position);
        chassis_set_position(rightFront,position);
        chassis_set_position(rightRear,position);
        chassis_left_set_power(power);
        chassis_right_set_power(power);

    }

    /**
     * Sets the left drive motors' power
     *
     * @param power  a Double that is the power for the left motors
     */
    //sets power to left side of robot to desired power value
    public void chassis_left_set_power(double power)
    {
        chassis_set_power(power, leftFront);
        chassis_set_power(power, leftRear);
    }

    /**
     * sets the right drive motors' power
     *
     * @param power  a Double that is the power for the right motors
     */
    //sets power to right side of robot to desired power value
    public void chassis_right_set_power(double power)
    {
        chassis_set_power(power, rightFront);
        chassis_set_power(power, rightRear);
    }

    //Direction is either l "L" for left or r for right, instead of F for forward and B for backward

    /**
     * A method for making point turns
     *
     * @param mode  a String that is the motor mode
     * @param power  a Double that is the power
     * @param direction  a String that is the direction of movement
     * @param position  an Int that is the motors' position on the robot
     */
    public void chassis_pointTurn(String mode, double power, String direction, int position){
        position= chassis_distance2encoder(position,6,1);
        if (direction.toLowerCase().equals("r")) {
            chassis_set_direction(leftFront, "f");
            chassis_set_direction(leftRear, "r");
            chassis_set_direction(rightFront, "f");
            chassis_set_direction(rightRear, "f");
        } else {
            chassis_set_direction(leftFront, "r");
            chassis_set_direction(leftRear, "f");
            chassis_set_direction(rightFront, "r");
            chassis_set_direction(rightRear, "r");
        }
        //sets mode to what is sent in with the "mode" parameter
        chassis_set_mode(leftFront, mode);
        chassis_set_mode(leftRear, mode);
        chassis_set_mode(rightFront, mode);
        chassis_set_mode(rightRear, mode);
        //sets target position to parameter "position"
        chassis_set_position(leftFront, position);
        chassis_set_position(leftRear,position);
        chassis_set_position(rightFront,position);
        chassis_set_position(rightRear,position);

        chassis_left_set_power(power);
        chassis_right_set_power(power);
    }

    /**
     * a method used for sliding sideways
     *
     * @param mode  a String that is the motors' mode
     * @param power  a Double that is the power
     * @param direction  a String that is the direction of movement
     * @param position  an Int that is the motors' position on the robot
     */
    //slides sideways until the wheels reach the desired encoder count
    public void chassis_slide_sideways(String mode, double power, String direction, int position){
        position = chassis_distance2encoder(position, 4, 1);
        position=position*2; //because the wheels on the mecanum wheels are at 45', multiply the encoder counts by 2
        chassis_set_mode(leftFront, mode);
        chassis_set_mode(leftRear, mode);
        chassis_set_mode(rightFront, mode);
        chassis_set_mode(rightRear, mode);
        if (direction.toLowerCase().equals("r")) {
            chassis_set_direction(leftFront, "f");
            chassis_set_direction(rightRear, "r");
            chassis_set_direction(rightFront, "f");
            chassis_set_direction(leftRear, "f");
            chassis_set_position(rightFront, position);
            chassis_set_position(rightRear, position);
            chassis_set_position(leftFront, position);
            chassis_set_position(leftRear, position);
            chassis_set_power(power, rightRear);
            chassis_set_power(power, rightFront);
            chassis_set_power(power, leftFront);
            chassis_set_power(power, leftRear);

        } else if (direction.toLowerCase().equals("l")) {  // added else tim
            chassis_set_direction(leftFront, "r");
            chassis_set_direction(rightRear, "f");
            chassis_set_direction(rightFront, "r");
            chassis_set_direction(leftRear, "r");
            chassis_set_position(rightFront, position);
            chassis_set_position(rightRear, position);
            chassis_set_position(leftFront, position);
            chassis_set_position(leftRear, position);
            chassis_set_power(power, rightRear);
            chassis_set_power(power, rightFront);
            chassis_set_power(power, leftFront);
            chassis_set_power(power, leftRear);
        }
    }


    public void chassis_driveStraightGyro(double power, double sensitivity, double target, boolean extGyro ){

        if(power>0) {
            chassis_set_direction(leftFront, "f");
            chassis_set_direction(leftRear, "r");
            chassis_set_direction(rightFront, "r");
            chassis_set_direction(rightRear, "r");
        }
        if(power<0) {
            chassis_set_direction(leftFront, "r");
            chassis_set_direction(leftRear, "f");
            chassis_set_direction(rightFront, "f");
            chassis_set_direction(rightRear, "f");
        }

        chassis_set_mode(leftFront, "RUE");
        chassis_set_mode(leftRear, "RUE");
        chassis_set_mode(rightFront, "RUE");
        chassis_set_mode(rightRear, "RUE");
        double heading = chassis_getHeadingDbl();

        if ( extGyro )
        {
            if ( heading < 0.0 )
            {
                heading = heading + 360.0;
            }
        }
        double correction = driveControl.getOutput( heading, target );

        if(power>0) {
            chassis_left_set_power(power + ((heading - target) / sensitivity));
            chassis_right_set_power(power - ((heading - target) / sensitivity));
//            chassis_left_set_power( power - correction );
//            chassis_right_set_power( power + correction );
        }
        if(power<0) {
//            chassis_left_set_power( Math.abs(power) + correction );
//            chassis_right_set_power( Math.abs(power) - correction );
            chassis_left_set_power(Math.abs(power) - ((heading - target) / sensitivity));
            chassis_right_set_power(Math.abs(power) + ((heading - target) / sensitivity));
        }
    }

//    public void motorTelemetry(DcMotor motor)
//    {
//        if(motor != null)
//        {
//            telemetry.addData("00", get_direction(motor));
//            telemetry.addData("01", get_mode(motor));
//            telemetry.addData("02", get_power_tele(motor));
//            telemetry.addData("03", get_position_tele(motor));
//        }
//    }

}
