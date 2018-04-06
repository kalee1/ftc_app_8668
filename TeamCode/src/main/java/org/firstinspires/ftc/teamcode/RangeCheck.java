package org.firstinspires.ftc.teamcode;

/**
 * This class filters out noise values by only returning true is there are 3 consecutive values
 * that are within the target range.
 *
 * @author Error 404 Robotics
 */

public class RangeCheck {

    /** The maximum allowed target value */
    private double upperLimit;
    /** The minimum allowed target value */
    private double lowerLimit;
    /** The current number of acceptable values */
    private int length = 0;
    /** The requirred number of acceptable values to return true */
    private int lengthLimit;

    /**
     * Assigns the variables to
     *
     * @param limit  The number of times the incoming values must be within the acceptable range
     *               before returning true.
     * @param lower  The mimimum acceptable value.
     * @param upper  The maximum acceptable value.
     * */
    RangeCheck(double upper, double lower, int limit)
    {
        upperLimit = upper;
        lowerLimit = lower;
        lengthLimit = limit;
    }

    /**
     * Reads the incoming values (rangeNum), and if those values are within the acceptable range
     * (upperLimit and lowerLimit) for enough cycles, then return true, otherwise loop over and
     * over until those conditions are met.
     *
     * @param rangeNum The values coming in to the method from the range sensor
     * */
    public boolean checkRange (double rangeNum)
    {
        if ( rangeNum < upperLimit && rangeNum > lowerLimit)
        {
            length = 0;
        }
        else
        {
            length++;
        }

        if(length >= lengthLimit)
        {

            return true;
        }
        else
        {
            return false;
        }

    }

    public int getLength()
    {
        return length;
    }



}
