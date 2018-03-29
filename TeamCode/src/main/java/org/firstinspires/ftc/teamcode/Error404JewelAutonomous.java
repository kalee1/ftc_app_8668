package org.firstinspires.ftc.teamcode;

 /**
 *Error404JewelAutonomous extends <code>Error404_Hardware_Tier2</code> and contains the state
  * machine that steps through each step in the autonomous mission.
  *
  * This year's field presents a unique challenge for autonomous. The field is set up so that there
  * are four unique starting positions and your robot has to be able to start from any of them. We
  * could have four individual programs (one for each quadrant) but this is clumsy and opens us up
  * to bugs. Instead what we did is we made one main autonomous class that holds the framework code
  * that is the same for all four quadrants and then we have four quadrant-specific child classes
  * that supply the quadrant-specific movement vaules.
 *
 * @author Team 8668
 * @see Error404_Hardware_Tier2
 */
public class Error404JewelAutonomous extends Error404_Hardware_Tier2

{
    //////////////////////////////////////////////////////////////////////////////////
    /** This int variable sets the starting case value for the state machine (which step in the sequence to start with -- i.e. the first one or case 0) */
    private int state = 0;
    private int encoder=0;
    private double timer=0;
    /** Initializing the method used for driving from the balancing stone to the cryptobox. */
    protected int cryptoboxDriveDistance;
    /** Initializing the method used for turning the correct side of the robot to face the cryptobox. */
    protected int turnToCryptobox;
    /** Initializing the method used for sliding to align on a particular column of the cryptobox. */
    protected int cryptoboxSlide;
    /** A string that indicates which side of the field the robot is on ("blue" or "red") */
    protected int slideAwayFromTheCryptobox;
    protected int turnToPile;
    protected int driveToPile;
    protected int backToCryptobox;
    protected int slideBackToCryptobox;

    private String fieldSide;
    /** A string that indicates side location on the field ("front" or "back"). */
    private String sideLocation;

    public Error404JewelAutonomous()
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
     * during autonomous. In the cases where the robot will drive somewhere, there are no values
     * hard-coded in as these values will change depending on what quadrant of the field the robot
     * is starting in. Instead the Autonomous class sources the quadrant-specific movement values
     * from field position-dependant child classes. */
    @Override public void loop ()
    {
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
                if(updateFromVuforia(readCryptograph()))
                {
                    state= 7;
                }


                if(((int)(getRuntime()-timer))>2)
                {
                    updateFromVuforia("CENTER");
                    state=7;
                }
                break;


            case 7:  //Drive to Cryptobox
                if(cryptoboxDriveDistance!=0)
                {
                    //driveStraight("RUE", 0.3, "f", 0);
                    if(cryptoboxDriveDistance>0)
                    {
                        driveStraightCombo(0.7);
                    }
                    else
                    {
                        driveStraightCombo(-0.7);
                    }

                    if (leftFront.getCurrentPosition() - encoder > Math.abs(cryptoboxDriveDistance))
                    {
                        //slide_sideways("RUE", 0, "l", 0);
                        stopEverything();
                        setMultipleDirections("turn", "right");
                        state++;
                        encoder = leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("turn", "right");
                    state++;
                    encoder = leftFront.getCurrentPosition();
                }

                break;

            case 8:  //Face Cryptobox
                if(turnToCryptobox!=0)
                {
                    //pointTurn("RUE", 0.3, "r", 0);
                    pointTurnCombo(0.6);
                    if (Math.abs(getHeading()) > turnToCryptobox)
                    {
                        state++;
                        stopEverything();
                        setMultipleDirections("straight", "forward");
                        encoder = leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("slide", "left");
                    state++;
                    encoder = leftFront.getCurrentPosition();
                }
                break;

            case 9:  // Slide to Cryptobox
                if(cryptoboxSlide!=0)
                {
                    if(cryptoboxSlide>0)
                    {
                        slideSidewaysCombo(0.7);
                    }
                    else
                    {
                        slideSidewaysCombo(-0.7);
                    }

                    if(leftFront.getCurrentPosition()-encoder>Math.abs(cryptoboxSlide))
                    {
                        stopEverything();
                        setMultipleDirections("straight", "forward");
                        state++;
                        encoder=leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("straight", "forward");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 10:  //Drive into Cryptobox
                driveStraight("RUE",0.6,"f",0);
                if(leftFront.getCurrentPosition()-encoder>150)
                {
                    driveStraight("RUE",0,"r",0);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 11:  //Deploy Glyph
                glyphIntake("out");
                state++;
                break;

            case 12:  //Wait
                //if(leftGlyph.getCurrentPosition()>20)
                {
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 13:  // Back Up
                driveStraightCombo(-0.7);
                if(leftFront.getCurrentPosition()-encoder>130)
                {
                    driveStraight("RUE",0,"f",0);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                    glyphIntake("stop");
                }
                break;

            case 14: // Push Glyph in
                glyphIntake("outSlow");
                driveStraightCombo(0.3);
                if(leftFront.getCurrentPosition()-encoder>150)
                {
                    driveStraight("RUE",0,"r",0);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 15: //Back away from the cryptobox
                driveStraightCombo(-0.7);
                if(leftFront.getCurrentPosition()-encoder>50)
                {
                    driveStraight("RUE",0,"f",0);
                    state = 17;
                    encoder=leftFront.getCurrentPosition();
                }
                break;


            case 17:  //Face Glyph Pile
                if(turnToPile!=0)
                {
                    pointTurnCombo(0.6);
                    if (getHeading() < turnToPile && (getHeading() > 0))
                    {
                        state++;
                        stopEverything();
                        setMultipleDirections("straight", "forward");
                        encoder = leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("slide", "left");
                    state++;
                    encoder = leftFront.getCurrentPosition();
                }
                break;

            case 18:  //Drive into GLyph Pile
                if(driveToPile!=0)
                {

                    driveStraightCombo(0.7);


                    if (leftFront.getCurrentPosition() - encoder > Math.abs(driveToPile))
                    {
                        //slide_sideways("RUE", 0, "l", 0);
                        stopEverything();
                        setMultipleDirections("straight", "forward");
                        state++;
                        encoder = leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("straight", "forward");
                    state++;
                    encoder = leftFront.getCurrentPosition();
                }

                break;

            case 19:  //Collect Glyphs
                glyphIntake("in");
                driveStraightCombo(0.3);
                if(leftFront.getCurrentPosition()-encoder>150)
                {
                    driveStraight("RUE",0,"r",0);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 20:  //Drive back to cryptobox -- investigate using
                      //ultrasonic sensors to detect distance away from wall and cryptobox
                if(backToCryptobox!=0)
                {
                    if(backToCryptobox>0)
                    {
                        driveStraightCombo(-0.7);
                    }
                    else
                    {
                        driveStraightCombo(0.7);
                    }

                    if (leftFront.getCurrentPosition() - encoder > Math.abs(backToCryptobox))
                    {
                        //slide_sideways("RUE", 0, "l", 0);
                        stopEverything();
                        glyphIntake("stop");
                        setMultipleDirections("slide", "right");
                        state = 22;
                        encoder = leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("slide", "left");
                    state = 22;
                    encoder = leftFront.getCurrentPosition();
                }

                break;

//            case 21:  //Line up on cryptobox column -- investigate using ultrasonic sensor to dedect
//                      //distance away from wall and cryptobox as well as detecting cryptobox
//                if(slideBackToCryptobox!=0)
//                {
//                    if(slideBackToCryptobox>0)
//                    {
//                        slideSidewaysCombo(0.7);
//                    }
//                    else
//                    {
//                        slideSidewaysCombo(-0.7);
//                    }
//
//                    if(leftFront.getCurrentPosition()-encoder>Math.abs(slideBackToCryptobox))
//                    {
//                        stopEverything();
//                        setMultipleDirections("straight", "backwards");
//                        state++;
//                        encoder=leftFront.getCurrentPosition();
//                    }
//                }
//                else
//                {
//                    setMultipleDirections("straight", "backwards");
//                    state++;
//                    encoder=leftFront.getCurrentPosition();
//                }
//                break;

            case 22:
                pointTurnCombo(0.6);
                if (getHeading() < 0 )
                {
                    state++;
                    stopEverything();
                    setMultipleDirections("straight", "reverse");
                    encoder = leftFront.getCurrentPosition();
                }

                break;

            case 23:  //Drive into Cryptobox
                driveStraightCombo(-0.6);
                if(leftFront.getCurrentPosition()-encoder>75)
                {
                    timer = getRuntime();
                    driveStraight("RUE",0,"f",0);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;


            case 24:  //Dump Glyphs
                glyphter.setPosition(0.22);
                state++;
                break;

            case 25:  //Wait
                if(((int)(getRuntime()-timer))>2.5)
                {
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 26:  // Back Up
                driveStraightCombo(0.7);
                if(leftFront.getCurrentPosition()-encoder>130)
                {
                    driveStraight("RUE",0,"r",0);
                    glyphter.setPosition(.05);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 27: // Push Glyphs in
                driveStraightCombo(-0.3);
                if(leftFront.getCurrentPosition()-encoder>130)
                {
                    driveStraight("RUE",0,"f",0);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 28: //Back away from the cryptobox
                driveStraightCombo(0.7);
                if(leftFront.getCurrentPosition()-encoder>100)
                {
                    driveStraight("RUE",0,"f",0);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;



            default:
                glyphIntake("stop");
                break;


        }
        telemetry.addData("1. State: ", state);
        telemetry.addData("2. Gyro: ", getHeading());
        telemetry.addData("3. Camera:  ", camera.getVoltage());
        telemetry.addData("4. Left Front Position: ", leftFront.getCurrentPosition());
        telemetry.addData("5. Delta Position: ", encoder);
        telemetry.addData("6. Cryptobox Drive Distance: ", cryptoboxDriveDistance);
        telemetry.addData("7. Cryptobox Slide: ", cryptoboxSlide);
        telemetry.addData("Pattern: ", readCryptograph());




    } // loop
} //
