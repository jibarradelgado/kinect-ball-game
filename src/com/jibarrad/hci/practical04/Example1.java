package com.jibarrad.hci.practical04;

import java.io.File;

import SimpleOpenNI.*;
import processing.core.*;

public class Example1 extends PApplet {
	
	// declare SimpleOpenNI object
	SimpleOpenNI context;

	// PImage to hold incoming imagery
	PImage cam;
	
	int[] userMap;
	
	public void setup() {
	
	size(640, 480);
	context = new SimpleOpenNI(this);
	context.enableDepth();
	context.enableUser();
	context.setMirror(true);
	}
	
	public void draw() {
		background(0);
		  context.update();
		  
		  if(context.getNumberOfUsers() > 0) {
		    
		    userMap = context.userMap();
		    
		    loadPixels();
		    for(int i = 0; i < userMap.length; i++) {
		      
		      if (userMap[i] !=0) {
		        pixels[i] =color(200, 200, 200);
		      }
		    }
		    updatePixels();
		  }
	}
}
