package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class Chassis {
    /**
     * This motor is one of the drive motors that makes up the robot drivetrain.
     */
    protected DcMotor leftFront;
    /**
     * This motor is one of the drive motors that makes up the robot drivetrain.
     */
    protected DcMotor rightFront;
    /**
     * This motor is one of the drive motors that makes up the robot drivetrain.
     */
    protected DcMotor leftRear;
    /**
     * This motor is one of the drive motors that makes up the robot drivetrain.
     */
    protected DcMotor rightRear;
    /**
     * The Rev Expansion Hub's own gryo and should only be used during initialization.
     */
    protected IntegratingGyroscope gyro;


    private MiniPID turnControl;
    private MiniPID driveControl;
    private boolean moving = false;
    protected int initialEncoder = 0;
    private double initialTime = 0;
    private double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);


    public void init( HardwareMap hardwareMap, Telemetry telemetry )
    {
        try {
            leftFront = hardwareMap.dcMotor.get("leftFront");
        } catch (Exception p_exeception) {
            telemetry.addData("leftFront not found in config file", 0);
            leftFront = null;
        }
        try {
            rightFront = hardwareMap.dcMotor.get("rightFront");
            rightFront.setDirection(FORWARD);
        } catch (Exception p_exeception) {
            telemetry.addData("rightFront not found in config file", 0);
            rightFront = null;
        }
        try {
            leftRear = hardwareMap.dcMotor.get("leftRear");
        } catch (Exception p_exeception) {
            telemetry.addData("leftRear not found in config file", 0);
            leftRear = null;
        }
        try {
            rightRear = hardwareMap.dcMotor.get("rightRear");
        } catch (Exception p_exeception) {
            telemetry.addData("rightRear not found in config file", 0);
            rightRear = null;
        }
        try {
            gyro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        } catch (Exception p_exeception) {
            telemetry.addData("navx not found in config file", 0);
            gyro = null;
        }

        turnControl = new MiniPID( .01, 0, .018 );
        turnControl.setOutputLimits(1.0);
        turnControl.setOutputMin( 0.25 );
        turnControl.reset();

        driveControl = new MiniPID( .025, 0, 0 );
        driveControl.setOutputLimits(1.0);
        driveControl.setOutputMin( 0.25 );
        driveControl.reset();

    }

    /**
     * Used to get the robot's heading.
     *
     * @return the robtot's heading as an Int
     */
    public int getHeading() {
        return (int) getHeadingDbl();
    }

    /**
     * Used to get the robot's heading.
     *
     * @return the robtot's heading as an Int
     */
    public double getHeadingDbl() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }


    ////////////////////////////////////////
    // In these set power methods, the    //
    //method checks to see if the motor   //
    //is null. If so, it skips that motor.//
    //If it is not null, the power is set //
    //to that motor.                      //
    ////////////////////////////////////////

    /**
     * If the motor is valid, set the motor to a certain power
     *
     * @param power a double the is the power to be set to the motor
     * @param motor The motor whose power will be set.
     */
    public void setPower(double power, DcMotor motor) {
        if (motor != null) {
            motor.setPower(power);
        }
    }

    ///////////////////////////////////////////////////////
    // This set mode method uses two parameters:         //
    // motor and a 3-4 letter mode abbreviation.         //
    // If the motor is not null, the mode will be set to://
    //RTP= Run to Position       //////////////////////////
    //RUE= Run using encoders    //
    //RWOE= Run without encoders //
    ///////////////////////////////

    /**
     * If the motor is valid, set it with 3 different available modes (RUN_TO_POSITION,
     * RUN-USING_ENCODERS, RUN_WITHOUT_ENCODERS).
     *
     * @param motor     The motor whose modes will be set.
     * @param modetoset The different modes to be set
     */
    public void setMode(DcMotor motor, String modetoset) {
        modetoset = modetoset.toUpperCase();
        if (motor != null) {
            if (modetoset.equals("RTP")) {
                motor.setMode(RUN_TO_POSITION);
            }
            if (modetoset.equals("RUE")) {
                motor.setMode(RUN_USING_ENCODER);
            }
            if (modetoset.equals("RWOE")) {
                motor.setMode(RUN_WITHOUT_ENCODER);
            }
        }
    }


    ///////////////////////////////////////////////////////////
    //This set direction method takes two parameters: Motor  //
    // and direction. The direction is set as F for forward  //
    // and R for reversed. If the motor is not null, the     //
    //direction is set.                                      //
    ///////////////////////////////////////////////////////////

    /**
     * If the motor is valid, set the motor direction as either forward (f) or reverse (r).
     *
     * @param motor     The motor whose direction will be set
     * @param direction a String that sets the motor's direction.
     */
    public void setDirection(DcMotor motor, String direction) {
        if (motor != null) {
            direction = direction.toLowerCase();
            if (direction.equals("r")) {
                motor.setDirection(REVERSE);
            }
            if (direction.equals("f")) {
                motor.setDirection(DcMotor.Direction.FORWARD);
            }
        }
    }

    //////////////////////////////////////////
    //This method takes two parameters, one //
    //for the motor and one for the desired //
    //position. It then sets the position   //
    //to the motor if the motor is not null.//
    //////////////////////////////////////////

    /**
     * Tells the motor to go to a certain position.
     *
     * @param motor    The motor whose position will be set.
     * @param position the desired posistion for the motor to achieve.
     */
    public void setPosition(DcMotor motor, int position) {
        if (motor != null) {
            motor.setTargetPosition(position);
        }
    }

    //////////////////////////////////////////////////
    //In this method, you input the desired distance//
    //in inches, the wheel diameter, and the gear   //
    //ratio. This method will then calculate the    //
    //needed number of encoder ticks needed to      //
    //drive the distance input.                     //
    //////////////////////////////////////////////////

    /**
     * converts inches to encoder counts
     *
     * @param desiredDistance an Int that is the target distance
     * @param wheel_diameter  a Double that is your wheel diameter
     * @param gear_ratio      a Double that is the gear ratio
     * @return the needed number of encoder ticks to move the target amount of inches
     */
    public int distance2Encoder(int desiredDistance, double wheel_diameter, double gear_ratio)
    {
        return (int) (280 * (desiredDistance / (((3.14159265) * (wheel_diameter)) * gear_ratio)));
    }


    /**
     * this method is used to stop all the drive motors.
     */
    public void stopEverything()
    {
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
        turnControl.reset();
        driveControl.reset();
        moving = false;
    }

    /**
     * Combines direction and power into one drive method that drives forward and backward.
     *
     * @param power determines the power which the motors run at
     */
    public void driveStraight(double power)
    {
        if (power > 0) {
            driveStraightImpl("RUE", power, "f", 0);
        } else if (power < 0) {
            driveStraightImpl("RUE", (power * (-1)), "r", 0);
        } else if (power == 0) {
            stopEverything();
        }
    }

    public boolean driveStraightDistance( double power, int encoderDistance )
    {
        if ( !moving )
        {
            // First call of this method for a move.  Set up the direction parameters and
            // grab the initial encoder position to use as a reference for distance.
            //
            if ( power > 0 )
            {
                setMultipleDirections("straight", "forward");
            }
            else
            {
                setMultipleDirections("straight", "reverse");
            }
            initialEncoder = leftFront.getCurrentPosition();
            moving = true;

        }
        if ( moving && ( leftFront.getCurrentPosition() - initialEncoder < Math.abs(encoderDistance) ) )
        {
            // Not moved far enough... continue moving.
            driveStraight( power );
        }
        else
        {
            // Finished the move.  Now stop.
            stopEverything();
            moving = false;
        }
        return !moving;
    }

    public boolean driveStraightDistanceGyro( double power, int encoderDistance, double targetHeading )
    {
        if ( !moving )
        {
            // First call of this method for a move.  Set up the direction parameters and
            // grab the initial encoder position to use as a reference for distance.
            //
            if ( power > 0 )
            {
                setMultipleDirections("straight", "forward");
            }
            else
            {
                setMultipleDirections("straight", "reverse");
            }
            initialEncoder = leftFront.getCurrentPosition();
            moving = true;

        }
        if ( moving && ( leftFront.getCurrentPosition() - initialEncoder < Math.abs(encoderDistance) ) )
        {
            // Not moved far enough... continue moving.
            driveStraightGyro( power, targetHeading );
        }
        else
        {
            // Finished the move.  Now stop.
            stopEverything();
            moving = false;
        }
        return !moving;
    }

    public boolean driveStraightTimed( double power, double duration )
    {
        if ( !moving )
        {
            // First call of this method for a move.  Set up the direction parameters and
            // grab the initial encoder position to use as a reference for distance.
            //
            if ( power > 0 )
            {
                setMultipleDirections("straight", "forward");
            }
            else
            {
                setMultipleDirections("straight", "reverse");
            }
            initialTime = System.nanoTime() / NANOSECONDS_PER_SECOND;
            moving = true;

        }
        if ( moving &&  (System.nanoTime()/NANOSECONDS_PER_SECOND - initialTime) < Math.abs(duration) )
        {
            // Not moved long enough... continue moving.
            driveStraight( power );
        }
        else
        {
            // Finished the move.  Now stop.
            stopEverything();
            moving = false;
        }
        return !moving;
    }


    /**
     * Combines direction and power into one drive method that slides left and right.
     *
     * @param power determines the power which the motors run at
     */
    public void slideSideways(double power)
    {
        if (power > 0)
        {
            slideSidewaysImpl("RUE", power, "r", 0);
        }
        else if (power < 0)
        {
            slideSidewaysImpl("RUE", (power * (-1)), "l", 0);
        }
        else if (power == 0)
        {
            stopEverything();
        }
    }

    public boolean slideSidewaysDistance( double power, int encoderDistance )
    {
        if ( !moving )
        {
            // First call of this method for a move.  Set up the direction parameters and
            // grab the initial encoder position to use as a reference for distance.
            //
            if ( power > 0 )
            {
                setMultipleDirections("slide", "right");
            }
            else
            {
                setMultipleDirections("slide", "left");
            }
            initialEncoder = leftFront.getCurrentPosition();
            moving = true;

        }
        if ( moving && ( leftFront.getCurrentPosition() - initialEncoder < Math.abs(encoderDistance) ) )
        {
            // Not moved far enough... continue moving.
            slideSideways( power );
        }
        else
        {
            // Finished the move.  Now stop.
            stopEverything();
            moving = false;
        }
        return !moving;

    }

    /**
     * Combines direction and power into one turning method that spins the robot left and right.
     *
     * @param power determines the power which the motors run at
     */
    public void pointTurn(double power) {
        if (power > 0)
        {
            pointTurnImpl("RUE", power, "r", 0);
        }
        else if (power < 0)
        {
            pointTurnImpl("RUE", (power * (-1)), "l", 0);
        }
        else if (power == 0)
        {
            stopEverything();
        }
    }

    /**
     * Uses PID control to turn to a particular heading.
     *
     * @param targetHeading specifies the direction the robot needs to face
     */
    public boolean pointTurnGyro(double targetHeading, double timeout, boolean extendedGyro)
    {

        double currentHeading = getHeadingDbl();
        if ( !moving )
        {
            if ( (currentHeading - targetHeading) < 0 )
            {
                setMultipleDirections("turn", "right");
            }
            else
            {
                setMultipleDirections("turn", "left");
            }
            initialTime = System.nanoTime() / NANOSECONDS_PER_SECOND;
            moving = true;
        }
        if (extendedGyro) {
            if (currentHeading < 0.0) {
                currentHeading = 360.0 + currentHeading;
            }
        }

        double motor_power = 0;

        if ( Math.abs(targetHeading) < 180.0 &&
             moving &&
             ( Math.abs(currentHeading - targetHeading) > 4.0 ||
               System.nanoTime() / NANOSECONDS_PER_SECOND - initialTime < timeout) )
        {
            // If my command is good AND I am moving AND
            // (not reached the target yet OR if the timeout hasn't tripped ), then keep moving
            motor_power = turnControl.getOutput(currentHeading, targetHeading);
            pointTurn(-1 * motor_power);
        }
        else
        {
            stopEverything();
            moving = false;
        }

        return !moving;
    }

    /**
     * Uses several parameters to set the motors the the directions needed for more complicated maneuvers.
     *
     * @param movement  specifies type of movement (i.e., straight, turn, slide)
     * @param direction sets motor directions for the encoders
     */
    protected void setMultipleDirections(String movement, String direction) {
        if (movement.equals("straight")) {
            if (direction.equals("forward")) {
                setDirection(leftFront, "f");
                setDirection(leftRear, "r");
                setDirection(rightFront, "r");
                setDirection(rightRear, "r");
            }
            if (direction.equals("reverse")) {
                setDirection(leftFront, "r");
                setDirection(leftRear, "f");
                setDirection(rightFront, "f");
                setDirection(rightRear, "f");
            }
        }
        if (movement.equals("turn")) {
            if (direction.equals("right")) {
                setDirection(leftFront, "f");
                setDirection(leftRear, "r");
                setDirection(rightFront, "f");
                setDirection(rightRear, "f");
            }
            if (direction.equals("left")) {
                setDirection(leftFront, "r");
                setDirection(leftRear, "f");
                setDirection(rightFront, "r");
                setDirection(rightRear, "r");
            }
        }
        if (movement.equals("slide")) {
            if (direction.equals("right")) {
                setDirection(leftFront, "f");
                setDirection(rightRear, "r");
                setDirection(rightFront, "f");
                setDirection(leftRear, "f");
            }
            if (direction.equals("left")) {
                setDirection(leftFront, "r");
                setDirection(rightRear, "f");
                setDirection(rightFront, "r");
                setDirection(leftRear, "r");
            }
        }
    }

    public int getCurrentPosition()
    {
        return leftFront.getCurrentPosition();
    }

    /**
     * Uses various parameters to make the robot drive straight
     *
     * @param mode      a String that gives the motor mode
     * @param power     a Double that gives the power
     * @param direction a String that tells the direction (forward "f" or reverse "r")
     * @param position  an Int that tells the motor position on the robot
     */
    //tells robot to drive straight at a certain power in a direction until the desired position is reached
    protected void driveStraightImpl(String mode, double power, String direction, int position)
    {
        position = distance2Encoder(position, 6, 1);

        if (direction.toLowerCase().equals("f"))
        {
            setDirection(leftFront, "f");
            setDirection(leftRear, "r");
            setDirection(rightFront, "r");
            setDirection(rightRear, "r");
        }
        else
        {
            setDirection(leftFront, "r");
            setDirection(leftRear, "f");
            setDirection(rightFront, "f");
            setDirection(rightRear, "f");
        }
        setMode(leftFront, mode);
        setMode(leftRear, mode);
        setMode(rightFront, mode);
        setMode(rightRear, mode);
        setPosition(leftFront, position);
        setPosition(leftRear, position);
        setPosition(rightFront, position);
        setPosition(rightRear, position);
        left_set_power(power);
        right_set_power(power);
        moving = true;

    }

    /**
     * Sets the left drive motors' power
     *
     * @param power a Double that is the power for the left motors
     */
    //sets power to left side of robot to desired power value
    protected void left_set_power(double power) {
        setPower(power, leftFront);
        setPower(power, leftRear);
    }

    /**
     * sets the right drive motors' power
     *
     * @param power a Double that is the power for the right motors
     */
    //sets power to right side of robot to desired power value
    protected void right_set_power(double power) {
        setPower(power, rightFront);
        setPower(power, rightRear);
    }

    //Direction is either l "L" for left or r for right, instead of F for forward and B for backward

    /**
     * A method for making point turns
     *
     * @param mode      a String that is the motor mode
     * @param power     a Double that is the power
     * @param direction a String that is the direction of movement
     * @param position  an Int that is the motors' position on the robot
     */
    protected void pointTurnImpl(String mode, double power, String direction, int position) {
        position = distance2Encoder(position, 6, 1);
        if (direction.toLowerCase().equals("r")) {
            setDirection(leftFront, "f");
            setDirection(leftRear, "r");
            setDirection(rightFront, "f");
            setDirection(rightRear, "f");
        } else {
            setDirection(leftFront, "r");
            setDirection(leftRear, "f");
            setDirection(rightFront, "r");
            setDirection(rightRear, "r");
        }
        //sets mode to what is sent in with the "mode" parameter
        setMode(leftFront, mode);
        setMode(leftRear, mode);
        setMode(rightFront, mode);
        setMode(rightRear, mode);
        //sets target position to parameter "position"
        setPosition(leftFront, position);
        setPosition(leftRear, position);
        setPosition(rightFront, position);
        setPosition(rightRear, position);

        left_set_power(power);
        right_set_power(power);
    }

    /**
     * a method used for sliding sideways
     *
     * @param mode      a String that is the motors' mode
     * @param power     a Double that is the power
     * @param direction a String that is the direction of movement
     * @param position  an Int that is the motors' position on the robot
     */
    //slides sideways until the wheels reach the desired encoder count
    protected void slideSidewaysImpl(String mode, double power, String direction, int position) {
        position = distance2Encoder(position, 4, 1);
        position = position * 2; //because the wheels on the mecanum wheels are at 45', multiply the encoder counts by 2
        setMode(leftFront, mode);
        setMode(leftRear, mode);
        setMode(rightFront, mode);
        setMode(rightRear, mode);
        if (direction.toLowerCase().equals("r")) {
            setDirection(leftFront, "f");
            setDirection(rightRear, "r");
            setDirection(rightFront, "f");
            setDirection(leftRear, "f");
            setPosition(rightFront, position);
            setPosition(rightRear, position);
            setPosition(leftFront, position);
            setPosition(leftRear, position);
            setPower(power, rightRear);
            setPower(power, rightFront);
            setPower(power, leftFront);
            setPower(power, leftRear);

        } else if (direction.toLowerCase().equals("l")) {  // added else tim
            setDirection(leftFront, "r");
            setDirection(rightRear, "f");
            setDirection(rightFront, "r");
            setDirection(leftRear, "r");
            setPosition(rightFront, position);
            setPosition(rightRear, position);
            setPosition(leftFront, position);
            setPosition(leftRear, position);
            setPower(power, rightRear);
            setPower(power, rightFront);
            setPower(power, leftFront);
            setPower(power, leftRear);
        }
    }


    public void driveStraightGyro(double power, double target ) {

        if (power > 0) {
            setDirection(leftFront, "f");
            setDirection(leftRear, "r");
            setDirection(rightFront, "r");
            setDirection(rightRear, "r");
        }
        if (power < 0) {
            setDirection(leftFront, "r");
            setDirection(leftRear, "f");
            setDirection(rightFront, "f");
            setDirection(rightRear, "f");
        }

        setMode(leftFront, "RUE");
        setMode(leftRear, "RUE");
        setMode(rightFront, "RUE");
        setMode(rightRear, "RUE");

        double heading = getHeadingDbl();

        if ( target > 90.0 && heading < 0.0 )
        {
            heading = heading + 360.0;
        }
        else if ( target < -90.0 && heading > 0.0 )
        {
            heading = heading - 360.0;
        }

        double correction = driveControl.getOutput(heading, target);

        if (power > 0) {
//            left_set_power(power + ((heading - target) / sensitivity));
//            right_set_power(power - ((heading - target) / sensitivity));
            left_set_power( power - correction );
            right_set_power( power + correction );
        }
        if (power < 0) {
            left_set_power( Math.abs(power) + correction );
            right_set_power( Math.abs(power) - correction );
//            left_set_power(Math.abs(power) - ((heading - target) / sensitivity));
//            right_set_power(Math.abs(power) + ((heading - target) / sensitivity));
        }
    }

}

