package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 *Error404AutonomousRear extends <code>Error404_Hardware_Tier2</code> and contains the state
  * machine that steps through each step in the autonomous mission.
  *
 * This year's field presents a unique challenge for autonomous. The field is set up so that there
 * are four unique starting positions and your robot has to be able to start from any of them. We
 * could have four individual programs (one for each quadrant) but this is clumsy and opens us up
 * to bugs. Instead what we did, is we made two main autonomous classes that hold the framework code
 * for the front two qudrants and the rear two quadrants. Then we have four quadrant-specific child
 * classes that supply the quadrant-specific movement vaules.
 *
 * @author Team 8668
 * @see Error404_Hardware_Tier2
 */
public class Error404AutonomousRear extends Error404_Hardware_Tier2

{
    //////////////////////////////////////////////////////////////////////////////////
    /** This int variable sets the starting case value for the state machine (which step in the sequence to start with -- i.e. the first one or case 0) */
    private int state = 0;
    /** This int variable sets the starting encoder values for the motors. */
    private int encoder=0;
    /** This double initializes a timer that can be used during the autonomous. */
    private double timer=0;
    /** The distance for driving from the glyph pile to the cryptobox. */
    protected int cryptoboxDriveDistance;
    /** The distance for sliding to line up on the glyph pile. */
    protected int glyphPileSlideDistance;
    /** The distance to drive the robot into the glyph pile. */
    protected int driveIntoPile;
    /** The angle on the gyro needed to turn to face the cryptobox. */
    protected int turnToCryptobox;
    /** The angle ont he gyro needed to turn to face the glyph pile */
    protected int turnToPile;
    /** The distance needed to line up on a perticular column of the cryptobox. */
    protected int cryptoboxSlide;
    /** A delta used when backing out of the glyph pile. Helps with minimizing encoder error. */
    protected int pileDelta;
    /** In terms of cm, how far away the rear of the robot is from the field wall. */
    protected double distanceFromWall;
    /** In terms of cm, how far away the robot is from the edge of the cryptobox. */
    protected int slideToEdge;
    /**
     * useExtendedGyro, if True, will be used in the gyro-controlled turn and drive methods to take
     * the navX +/- 180 degree gyro and read it like a 360 degree gyro. This lets the robot drive at
     * or turn to a heading of +/- 180 degrees without erroring out.
     *
     * If False, useExtendedGyro will not be used in the turn and drive methods and the hyro will be read normally.
     *  */
    protected boolean useExtendedGyro;

    /** A string that indicates which side of the field the robot is on ("blue" or "red") */
    private String fieldSide;
    /** A string that indicates side location on the field ("front" or "back"). */
    private String sideLocation;

    private String theColumn = "NONE";

    protected RangeCheck distanceCheckBottom = new RangeCheck(360, 52, 3);

    protected RangeCheck distanceCheckTop = new RangeCheck(360, 20, 3);

    public Error404AutonomousRear()
    {
    }

    //////////////////////////////////////////////////////////////////////////////////
   /** Gets the gyro heading, sets the jewel servos to the home poition, and grabs the encoder
    * value of the leftFront drive motor for navigation purposes. */
    @Override public void init(){
        super.init();
        telemetry.addData("Gyro: ", getHeading());
        telemetry.addData("","V 1");
        arm.setPosition(0.1);
        swivel.setPosition(0.52);
        encoder=leftFront.getCurrentPosition();

        encoder=leftFront.getCurrentPosition();
    }


    //////////////////////////////////////////////////////////////////////////////////
    /** Determins the field position that robot is starting in and utilizes the coorisponding
     * child class to get the move values for that quadrant of the field. */
    @Override public void start(){
        super.start();
    }

    public void setLocation(String frontOrBack, String redOrBlue){
        fieldSide = redOrBlue;
        sideLocation = frontOrBack;
    }

    protected boolean updateFromVuforia(String cryptoboxKey)
    {
        return true;
    }


    //////////////////////////////////////////////////////////////////////////////////
    /** Contains the state machine which contains the structure for each move the robot will take
     * during autonomous. In the cases where the robot will drive or turn, there are no
     * quadrant-specific values hard-coded in as these values will change depending on what quadrant
     * of the field the robot is starting in. Instead the Autonomous class sources the
     * quadrant-specific movement and turn values from field position-dependant child classes. */
    @Override public void loop ()
    {
        boolean result = false;

        switch (state)
        {
            case 0: // Lower Arm
                arm.setPosition(0.79);
                    state++;
                break;

            case 1:  //Find Jewel
                timer =getRuntime();
                if(camera.getVoltage()<2.1){
                    telemetry.addData("On left","");
                    if(fieldSide.equals("BLUE")){
                        state=3;
                    }
                    if(fieldSide.equals("RED")){
                        state=2;
                    }
                    break;
                }
                else if(camera.getVoltage()>2.1){
                    telemetry.addData("On right","");
                    if(fieldSide.equals("BLUE")){
                        state=2;
                    }
                    if(fieldSide.equals("RED")){
                        state=3;
                    }
                    break;
                }

            case 2:  //Swivel Right
                if(((int)(getRuntime()-timer))>0.3) {
                    swivel.setPosition(0.7);
                    state=4;
                    timer =getRuntime();
                }
                break;

            case 3:  //Swivel Left
                if(((int)(getRuntime()-timer))>0.3) {
                    swivel.setPosition(0.4);
                    state=4;
                    timer =getRuntime();
                }
                break;

            case 4:  //Reset Arm
                if(((int)(getRuntime()-timer))>0.1) {
                    arm.setPosition(0.1);
                    swivel.setPosition(0.5);
                    state=6;
                    timer =getRuntime();
                }
                break;

            case 6:  //Read Pictograph
                theColumn = readCryptograph();
                if( updateFromVuforia( theColumn ) ) {
                    state = 13;
                    if (fieldSide.equals("BLUE")) {
                        setMultipleDirections("straight", "reverse");
                    }
                    if (fieldSide.equals("RED")) {
                        setMultipleDirections("straight", "forward");
                    }
                }
                if(((int)(getRuntime()-timer))>2)
                {
                    updateFromVuforia("CENTER");
                    if (fieldSide.equals("BLUE")) {
                        setMultipleDirections("straight", "reverse");
                    }
                    if (fieldSide.equals("RED")) {
                        setMultipleDirections("straight", "forward");
                    }
                    state=13;
                }
                break;

            case 13:  // Drive To Cryptobox
                if (fieldSide.equals("BLUE")) {
                    driveStraightGyro(-0.5, 40, 0, useExtendedGyro);
                }
                if (fieldSide.equals("RED")) {
                    driveStraightGyro(0.5, 40, 0, useExtendedGyro);
                }
                if( Math.abs(leftFront.getCurrentPosition() - encoder) > Math.abs(cryptoboxDriveDistance) )  //need to make this generic
                {
                        stopEverything();
                        if(fieldSide.equals("RED")){
                            setMultipleDirections("slide", "left");
                        }
                        else{
                            setMultipleDirections("turn", "right");
                        }
                        state = 14;
                        encoder=leftFront.getCurrentPosition();
                }
                break;

            case 14:  // face Cryptobox
                if(fieldSide.equals("RED")) {
                    state = 15;
                    break;
                }
                if(fieldSide.equals("BLUE")) {
                    if ( pointTurnGyro(turnToCryptobox, true))
                    {
                        state = 15;
                        stopEverything();
                        setMultipleDirections("slide", "right");
                        encoder = leftFront.getCurrentPosition();
                    }
                }
                break;

            case 15:  // Line up on a cryptobox column
                if(cryptoboxSlide!=0)
                {
                    if(cryptoboxSlide>0)
                    {
                        slideSidewaysCombo(.6);
                    }
                    else
                    {
                        slideSidewaysCombo(-.6);
                    }

                    if(leftFront.getCurrentPosition()-encoder>Math.abs(cryptoboxSlide))
                    {
                        stopEverything();
                        setMultipleDirections("straight", "forward");
                        state++;
                        timer=getRuntime();
                        encoder=leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("straight", "forward");
                    state++;
                    timer=getRuntime();
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 16:  //Drive into Cryptobox
                driveStraightCombo(0.6);
                if(leftFront.getCurrentPosition()-encoder>70 || ((int)(getRuntime()-timer))>1.3)
                {
                    stopEverything();
                    setMultipleDirections("straight", "reverse");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 17:  //Deploy Glyph
                glyphIntake("out");
                timer=getRuntime();
                state++;
                break;

            case 18:  //Wait
                if(getRuntime()-timer>0.8)
                {
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 19:  // Back Up
                driveStraightCombo(-0.7);
                if(leftFront.getCurrentPosition()-encoder>70)
                {
                    stopEverything();
                    setMultipleDirections("straight", "forward");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                    glyphIntake("outSlow");
                }
                break;
            case 20: // Push Glyph in
                driveStraightCombo(0.5);
                if(leftFront.getCurrentPosition()-encoder>60)
                {
                    stopEverything();
                    setMultipleDirections("straight", "reverse");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;
            case 21:  //drive away from cryptobox into safe zone
                driveStraightCombo(-0.5);
                if(leftFront.getCurrentPosition()-encoder>40)
                {
                    stopEverything();
                    state++;
                    glyphIntake("stop");
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            default:
                break;


        }
        telemetry.addData("1. State: ", state);
        telemetry.addData("2. Gyro: ", getHeading());
        telemetry.addData("3. Camera:  ", camera.getVoltage());
        telemetry.addData("4. Left Front Position: ", leftFront.getCurrentPosition());
        telemetry.addData("5. Delta Position: ", encoder);
        telemetry.addData("6. Pattern: ", theColumn );
        telemetry.addData("7. slide distance ", cryptoboxSlide );

        //        telemetry.addData("6. Bottom wall Distance: ", bottomRange.getDistance(DistanceUnit.CM) );
//        telemetry.addData("8. rangeCheck result: ", result);
//        telemetry.addData("9. length: ", distanceCheckBottom.getLength());
//        telemetry.addData("7. Top Wall Distance: ", topRange.cmUltrasonic());




    } // loop
} //
