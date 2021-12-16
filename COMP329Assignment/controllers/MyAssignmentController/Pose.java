// File: Pose.java
// Date: 30th Dec 2020
// Description: Pose Class support for COMP329 Programming Assignment (2020)
// Author: Terry Payne
// Modifications:
//     Addition of getDeltaTheta() and update to the toString() method - 26th Nov 2021
/**
 * The Pose class for a robot 
 * Based on Worksheet 3 for COMP329, Nov 2020
 * 
 * @author Dr Terry R. Payne (trp@liv.ac.uk)
 *
 */
 
public class Pose {
  private double x;    // position on x axis - assume units are meters
  private double y;    // position on y axis - assume units are meters
  private double theta;  // This determines the angle (radians) anticlockwise from the x-axis line

  // ==================================================================================
  // Constructors
  // ==================================================================================
  public Pose() {
    this(0.0,0.0,0.0);
  }

  public Pose(double xpos, double ypos, double theta) {
    this.x = xpos;
    this.y = ypos;
    this.setTheta(theta);
  }

  public Pose(Pose p) {
    this.x = p.getX();
    this.y = p.getY();
    this.setTheta(p.getTheta());
  }

  // ==================================================================================
  // Getters / Setters  
  // ==================================================================================
  public void setTheta(double theta) {
    // Ensure that theta is in the range -\pi..\pi
    if (theta>Math.PI) {
      this.theta=(2 * Math.PI)-theta;
    } else if (theta < -Math.PI) {
      this.theta = (2 * Math.PI)+theta;
    } else {
      this.theta = theta;
    }
  }

  public void setPosition(double xpos, double ypos, double theta) {
    this.x = xpos;
    this.y = ypos;
    this.setTheta(theta);
  }

  public void setPosition(Pose p) {
    this.setPosition(p.getX(), p.getY(), p.getTheta());
  }

  public String toString() {
    return String.format("<%.03f", this.x) + ", " +
        String.format("%.03f", this.y) + ", " +
        String.format("%.03f", this.theta) +">";
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getTheta() {
    return theta;
  }
  
  // Find the difference in radians between some heading and the current pose
  public double getDeltaTheta(double theta) {
    double d = theta - this.theta;
    if (d > Math.PI)
      d = -(2*Math.PI) + d;
    else if (d < -Math.PI)
      d = (2*Math.PI) + d;
    return d;
  }

}