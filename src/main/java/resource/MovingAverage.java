/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package resource;

/**
 * package com.team254.lib.util;
 */

import java.util.ArrayList;

/**
 * Helper class for storing and calculating a moving average
 */
public class MovingAverage {

    ArrayList<Double> mNumbers = new ArrayList<Double>();
    int mSize;

    public MovingAverage(int pMaxSize) {
        this.mSize = pMaxSize;
    }

    public void addNumber(double pNewNumber) {
        mNumbers.add(pNewNumber);
        if (mNumbers.size() > mSize) {
            mNumbers.remove(0);
        }
    }

    public double getAverage() {
        double total = 0;

        for (double number : mNumbers) {
            total += number;
        }

        return mNumbers.size() == 0 ? 0 : total / mNumbers.size();
    }

    public int getSize() {
        return mNumbers.size();
    }

    public boolean isUnderMaxSize() {
        return getSize() < mSize;
    }

    public void clear() {
        mNumbers.clear();
    }

}