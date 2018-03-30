package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

    /** When the driver hits start sets up PID control. */
    @Override public void start()
    {
        turnControl = new MiniPID( .01, 0.00000, .018 );
//        turnControl = new MiniPID(Kp, 2*Kp/Pc, 0.33*Kp*Pc);
        turnControl.setOutputLimits(1.0);
//        turnControl.setOutputRampRate(0.25);
        turnControl.reset();
        super.start();
    }

    /** this method is used to stop all the drive motors. */
    public void stopEverything(){
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
    }

    /**
     * THe glyphIntake method triggers the two intake motors that will suck in or spit out a glyph.
     *
     * @param inOrOut  A string that determines the motor powers and direction. */
    public void glyphIntake(String inOrOut){
        if(inOrOut.toLowerCase().equals("outSlow")){
            rightGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightGlyph.setDirection(REVERSE);
            leftGlyph.setDirection(FORWARD);
            leftGlyph.setPower(0.2);
            rightGlyph.setPower(0.2);
        }
        if(inOrOut.toLowerCase().equals("out")){
            rightGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightGlyph.setDirection(REVERSE);
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
            rightGlyph.setDirection(FORWARD);
            leftGlyph.setDirection(REVERSE);
            leftGlyph.setPower(0.5);
            rightGlyph.setPower(0.5);
        }
    }

    /**
     * Combines direction and power into one drive method that drives forward and backward.
     *
     * @param power  determines the power which the motors run at*/
    public void driveStraightCombo(double power){
        if(power>0){
            driveStraight("RUE",power,"f",0);
        }
        else if(power<0){
            driveStraight("RUE",(power*(-1)),"r",0);
        }
        else if(power==0){
            stopEverything();
        }
    }

    /**
     * Combines direction and power into one drive method that slides left and right.
     *
     * @param power  determines the power which the motors run at */
    public void slideSidewaysCombo(double power){
        if(power>0){
            slide_sideways("RUE",power,"r",0);
        }
        else if(power<0){
            slide_sideways("RUE",(power*(-1)),"l",0);
        }
        else if(power==0){
            stopEverything();
        }
    }

    /**
     * Combines direction and power into one turning method that spins the robot left and right.
     *
     * @param power  determines the power which the motors run at */
    public void pointTurnCombo(double power){
        if(power>0){
            pointTurn("RUE",power,"r",0);
        }
        else if(power<0){
            pointTurn("RUE",(power*(-1)),"l",0);
        }
        else if(power==0){
            stopEverything();
        }
    }

    /**
     * Uses PID control to turn to a particular heading.
     *
     * @param targetHeading specifies the direction the robot needs to face
     */
    public boolean pointTurnGyro( double targetHeading )
    {
        boolean done = false;
        double currentHeading = getHeadingDbl();
        double motor_power = 0;

        if ( Math.abs( targetHeading ) > 180.0 )
        {
            // Reject any targetHeadings outside of the allowable rage of the gyro (+/- 180 degrees)
            done = true;
        }
        else
        {
            motor_power = turnControl.getOutput( currentHeading, targetHeading );
            pointTurnCombo( -1*motor_power );

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
    public void setMultipleDirections(String movement, String direction){
        if(movement.equals("straight")){
            if(direction.equals("forward")){
                set_direction(leftFront, "f");
                set_direction(leftRear, "r");
                set_direction(rightFront, "f");
                set_direction(rightRear, "r");
            }
            if(direction.equals("reverse")){
                set_direction(leftFront, "r");
                set_direction(leftRear, "f");
                set_direction(rightFront, "r");
                set_direction(rightRear, "f");
            }
        }
        if(movement.equals("turn")){
            if(direction.equals("right")){
                set_direction(leftFront, "f");
                set_direction(leftRear, "r");
                set_direction(rightFront, "r");
                set_direction(rightRear, "f");
            }
            if(direction.equals("left")){
                set_direction(leftFront, "r");
                set_direction(leftRear, "f");
                set_direction(rightFront, "f");
                set_direction(rightRear, "r");
            }
        }
        if(movement.equals("slide")){
            if(direction.equals("right")){
                set_direction(leftFront, "f");
                set_direction(rightRear, "r");
                set_direction(rightFront, "r");
                set_direction(leftRear, "f");
            }
            if(direction.equals("left")){
                set_direction(leftFront, "r");
                set_direction(rightRear, "f");
                set_direction(rightFront, "f");
                set_direction(leftRear, "r");
            }
        }
    }

    public void setServoPos(Servo servomotor, Double position){

        servomotor.setPosition(position);
    }

    public boolean ourColorOnRight(int buffer){
        return  true;
    }
    public void gyroCalibrate()
    {
        double before = getHeading();
        //gyro.calibrate();
        while (getHeading() != 0) {
            telemetry.addData("Gyro: ", getHeading());
        }
        telemetry.addData("Gyro Calibrated", "");
        telemetry.addData("Before: ", before);
        telemetry.addData("After: ", getHeading());

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
    public void driveStraight(String mode, double power, String direction, int position) {
        position=distance2encoder(position,6,1);
        if (direction.toLowerCase().equals("f")) {
            set_direction(leftFront, "f");
            set_direction(leftRear, "r");
            set_direction(rightFront, "f");
            set_direction(rightRear, "r");
        } else {
            set_direction(leftFront, "r");
            set_direction(leftRear, "f");
            set_direction(rightFront, "r");
            set_direction(rightRear, "f");
        }
        set_mode(leftFront, mode);
        set_mode(leftRear, mode);
        set_mode(rightFront, mode);
        set_mode(rightRear, mode);
        set_position(leftFront, position);
        set_position(leftRear,position);
        set_position(rightFront,position);
        set_position(rightRear,position);
        left_set_power(power);
        right_set_power(power);

    }

    /**
     * Sets the left drive motors' power
     *
     * @param power  a Duble that is the power for the left motors
     */
    //sets power to left side of robot to desired power value
    public void left_set_power(double power)
    {
        set_power(power, leftFront);
        set_power(power, leftRear);
    }

    /**
     * sets the right drive motors' power
     *
     * @param power  a Double that is the power for the right motors
     */
    //sets power to right side of robot to desired power value
    public void right_set_power(double power)
    {
        set_power(power, rightFront);
        set_power(power, rightRear);
    }
    /*public void resetAllEncoders_withWait(){
        int count=0;
        reset_encoder(rightFront);
        reset_encoder(rightRear);
        reset_encoder(leftFront);
        reset_encoder(leftRear);
        while (get_position(rightFront)!= 0 && get_position(rightRear)!= 0 && get_position(leftFront)!= 0 && get_position(leftRear)!= 0){
            count++;
            telemetry.addData("count: ", count);
        }
    }
    public void resetAllEncoders_noWait(){
        reset_encoder(rightFront);
        reset_encoder(rightRear);
        reset_encoder(leftFront);
        reset_encoder(leftRear);
    }
*/

    //Direction is either l "L" for left or r for right, instead of F for forward and B for backward

    /**
     * A method for making point turns
     *
     * @param mode  a String that is the motor mode
     * @param power  a Double that is the power
     * @param direction  a String that is the direction of movement
     * @param position  an Int that is the motors' position on the robot
     */
    public void pointTurn(String mode, double power, String direction, int position){
        position=distance2encoder(position,6,1);
        if (direction.toLowerCase().equals("r")) {
            set_direction(leftFront, "f");
            set_direction(leftRear, "r");
            set_direction(rightFront, "r");
            set_direction(rightRear, "f");
        } else {
            set_direction(leftFront, "r");
            set_direction(leftRear, "f");
            set_direction(rightFront, "f");
            set_direction(rightRear, "r");
        }
        //sets mode to what is sent in with the "mode" parameter
        set_mode(leftFront, mode);
        set_mode(leftRear, mode);
        set_mode(rightFront, mode);
        set_mode(rightRear, mode);
        //sets target position to parameter "position"
        set_position(leftFront, position);
        set_position(leftRear,position);
        set_position(rightFront,position);
        set_position(rightRear,position);

        left_set_power(power);
        right_set_power(power);
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
    public void slide_sideways(String mode, double power, String direction, int position){
        position = distance2encoder(position, 4, 1);
        position=position*2; //because the wheels on the mecanum wheels are at 45', multiply the encoder counts by 2
        set_mode(leftFront, mode);
        set_mode(leftRear, mode);
        set_mode(rightFront, mode);
        set_mode(rightRear, mode);
        if (direction.toLowerCase().equals("r")) {
            set_direction(leftFront, "f");
            set_direction(rightRear, "r");
            set_direction(rightFront, "r");
            set_direction(leftRear, "f");
            set_position(rightFront, position);
            set_position(rightRear, position);
            set_position(leftFront, position);
            set_position(leftRear, position);
            set_power(power, rightRear);
            set_power(power, rightFront);
            set_power(power, leftFront);
            set_power(power, leftRear);

        } else if (direction.toLowerCase().equals("l")) {  // added else tim
            set_direction(leftFront, "r");
            set_direction(rightRear, "f");
            set_direction(rightFront, "f");
            set_direction(leftRear, "r");
            set_position(rightFront, position);
            set_position(rightRear, position);
            set_position(leftFront, position);
            set_position(leftRear, position);
            set_power(power, rightRear);
            set_power(power, rightFront);
            set_power(power, leftFront);
            set_power(power, leftRear);
        }
    }
    public void slide_sideways_gyro(String mode, double power, String direction, int zeropoint){
        int maxDrift=5;
        int drift=0;
        int current=getHeading();
        set_mode(leftFront, mode);
        set_mode(leftRear, mode);
        set_mode(rightFront, mode);
        set_mode(rightRear, mode);
        if (direction.toLowerCase().equals("r")) {
            drift=(zeropoint-current);
            telemetry.addData("Drift in direction R: ",drift);
            if(Math.abs(drift)>=maxDrift){
                telemetry.addData("Max Drift Achieved","");
                left_set_power(0);
                right_set_power(0);
                if(drift>0 && current<180){
                    turn_gyro_power_new(zeropoint,0.2, 0.6, "r");
                    telemetry.addData("adjusting right","");
                }else {
                    turn_gyro_power_new(zeropoint,0.2, 0.6, "l");
                    telemetry.addData("adjusting left","");
                }
            }
            else {
                set_direction(leftFront, "f");
                set_direction(rightRear, "r");
                set_direction(rightFront, "r");
                set_direction(leftRear, "f");
                set_power(power, rightRear);
                set_power(power, rightFront);
                set_power(power, leftFront);
                set_power(power, leftRear);
            }

        } else {       // added else tim
            drift=(zeropoint-current);
            telemetry.addData("Drift in direction L: ",drift);
            if(Math.abs(drift)>=maxDrift){
                telemetry.addData("Max Drift Achieved","");
                left_set_power(0);
                right_set_power(0);
                if(drift>0 && current>180){
                    turn_gyro_power_new(zeropoint,0.2, 0.6, "l");
                    telemetry.addData("adjusting left","");
                }else {
                    turn_gyro_power_new(zeropoint,0.2, 0.6, "r");
                    telemetry.addData("adjusting right","");
                }
            }
            else {
                set_direction(leftFront, "r");
                set_direction(rightRear, "f");
                set_direction(rightFront, "f");
                set_direction(leftRear, "r");
                set_power(power, rightRear);
                set_power(power, rightFront);
                set_power(power, leftFront);
                set_power(power, leftRear);
            }
        }
    }


    /**
     * a method used for making gyro turns
     *
     * @param desired_gyro  an Int that is the target gyro heading
     * @param starting_power  a Double that is the starting power
     * @param fraction_to_change_power
     * @param direction  a String that is the direction of movement
     */
    //turns to a desired gyroscope position at a certain power
    public void turn_gyro_power_new(int desired_gyro, double starting_power, double fraction_to_change_power, String direction){
        double powervalue=0;
        int heading = getHeading();
        double last_part=(desired_gyro*fraction_to_change_power);
        if(direction.toLowerCase().equals("r")) {
            if (heading <= last_part) {
                powervalue = starting_power;
            }
            else if (heading> last_part) {
                powervalue = (desired_gyro - heading) / 50;
            }

            if(powervalue < 0.03) {
                powervalue = 0.03;
            }

            pointTurn("RUE", powervalue, "r", 0); //turn towards line
        } // end if direction = r
        else {
            if (heading>180)
            {
                heading=-360+heading;
            }
            telemetry.addData("Left Turn Heading: ",heading);

            if (heading >= -last_part) {
                powervalue = starting_power;
            }
            else// if (heading < -last_part)
            {
                powervalue = (desired_gyro - Math.abs(heading)) / 50;
            }

            if (powervalue < 0.03) {
                powervalue = 0.03;
            }

            pointTurn("RUE", powervalue, "l", 0); //turn towards line
        } // end if direction = l

    }
    public void turn_gyro_power(int desired_gyro, double starting_power, double fraction_to_change_power, String direction){
        double powervalue=0;
        double last_part=(desired_gyro*fraction_to_change_power);
        if(direction.toLowerCase()=="r") {
            if (getHeading() <= last_part) {
                powervalue = starting_power;
            }
            if (getHeading() > last_part && getHeading() < desired_gyro) {
                powervalue = (desired_gyro - getHeading()) / 200;

            }
            if(powervalue <0.03) {
                powervalue = 0.03;
            }

            pointTurn("RUE", powervalue, "r", 0); //turn towards line
        } // end if direction = r
        if(direction.toLowerCase()=="l") {
            int heading = getHeading();
            if (heading>180)
            {
                heading=-360+heading;
            }
            if (heading > last_part) {
                powervalue = starting_power;
            }
            if (heading < last_part && heading > desired_gyro) {
                powervalue = (desired_gyro - getHeading()) / 200;
            }
            if (powervalue < 0.03) {
                powervalue = 0.03;
            }

            pointTurn("RUE", powervalue, "l", 0); //turn towards line
        } // end if direction = l

    }
    public double ramp_up(double powerBegin, double powerEnd, double powerToWrite){
        if (powerBegin<powerEnd)
        {
            powerToWrite+=0.001;
        }
        else if (powerBegin>powerEnd)
        {
            powerToWrite-=0.001;
        }
        else if (powerEnd==0)
        {
            powerToWrite=0;
        }
        return powerToWrite;
    }

    double rampUpMethod (double motorOut, double analogIn, double slewRate) {
        if (slewRate < (Math.abs(motorOut - analogIn))) {
            if (motorOut - analogIn < 0) return (motorOut + slewRate);
            else return (motorOut - slewRate);
        }
        else return analogIn;
    }
    
    public void gyroStraightTarget (double power, double sensitivity, double target){
        set_direction(leftFront, "f");
        set_direction(leftRear, "r");
        set_direction(rightFront, "f");
        set_direction(rightRear, "r");

        set_mode(leftFront, "RUE");
        set_mode(leftRear, "RUE");
        set_mode(rightFront, "RUE");
        set_mode(rightRear, "RUE");
        double heading = (double)getHeading();
        left_set_power(power+((heading-target)/sensitivity));
        right_set_power(power-((heading-target)/sensitivity));
    }

    public void motorTelemetry(DcMotor motor)
    {
        if(motor != null)
        {
            telemetry.addData("00", get_direction(motor));
            telemetry.addData("01", get_mode(motor));
            telemetry.addData("02", get_power_tele(motor));
            telemetry.addData("03", get_position_tele(motor));
        }
    }

}
