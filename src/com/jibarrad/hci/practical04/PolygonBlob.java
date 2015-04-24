package com.jibarrad.hci.practical04;

import java.util.ArrayList;
import java.util.Collections;

import processing.core.*; //Processing core
import blobDetection.*; // blobs
import toxi.geom.*; // toxiclibs shapes and vectors

import org.jbox2d.collision.shapes.*; // jbox2d
import org.jbox2d.common.*; // jbox2d
import org.jbox2d.dynamics.*; // jbox2d

//an extended polygon class quite similar to the earlier PolygonBlob class (but extending Toxiclibs' Polygon2D class instead)
//The main difference is that this one is able to create (and destroy) a box2d body from it's own shape
public class PolygonBlob extends Polygon2D {
	//to hold the box2d body
	Body body;
	
	Example2 p;
	
	public PolygonBlob(Example2 p) {
		this.p = p;
	}

	// the createPolygon() method is nearly identical to the one presented earlier
	  // see the Kinect Flow Example for a more detailed description of this method (again, feel free to improve it)
	public void createPolygon() {
		ArrayList<ArrayList<PVector>> contours = new ArrayList<ArrayList<PVector>>();
		int selectedContour = 0;
		int selectedPoint = 0;

		// create contours from blobs
		for (int n=0 ; n<p.theBlobDetection.getBlobNb(); n++) {
			Blob b = p.theBlobDetection.getBlob(n);
			if (b != null && b.getEdgeNb() > 100) {
				ArrayList<PVector> contour = new ArrayList<PVector>();
				for (int m=0; m<b.getEdgeNb(); m++) {
					EdgeVertex eA = b.getEdgeVertexA(m);
					EdgeVertex eB = b.getEdgeVertexB(m);
					if (eA != null && eB != null) {
						EdgeVertex fn = b.getEdgeVertexA((m+1) % b.getEdgeNb());
						EdgeVertex fp = b.getEdgeVertexA((Math.max(0, m-1)));
						//dist(x1,y1,x2,y2) translated into Math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
						float dn = (float) Math.sqrt((fn.x*p.kinectWidth-eA.x*p.kinectWidth)
								*(fn.x*p.kinectWidth-eA.x*p.kinectWidth) 
								+(fn.y*p.kinectHeight-eA.y*p.kinectHeight)
								*(fn.y*p.kinectHeight-eA.y*p.kinectHeight));
						float dp = (float) Math.sqrt((fp.x*p.kinectWidth-eA.x*p.kinectWidth)
								*(fp.x*p.kinectWidth-eA.x*p.kinectWidth) 
								+(fp.y*p.kinectHeight-eA.y*p.kinectHeight)
								*(fp.y*p.kinectHeight-eA.y*p.kinectHeight));
						//float dn = dist(eA.x*p.kinectWidth, eA.y*p.kinectHeight, fn.x*p.kinectWidth, fn.y*p.kinectHeight);
						//float dp = dist(eA.x*p.kinectWidth, eA.y*p.kinectHeight, fp.x*p.kinectWidth, fp.y*p.kinectHeight);
						if (dn > 15 || dp > 15) {
							if (contour.size() > 0) {
								contour.add(new PVector(eB.x*p.kinectWidth, eB.y*p.kinectHeight));
								contours.add(contour);
								contour = new ArrayList<PVector>();
							} else {
								contour.add(new PVector(eA.x*p.kinectWidth, eA.y*p.kinectHeight));
							}
						} else {
							contour.add(new PVector(eA.x*p.kinectWidth, eA.y*p.kinectHeight));
						}
					}
				}
			}
		}

		while (contours.size() > 0) {

			// find next contour
			float distance = 999999999;
			if (getNumPoints() > 0) {
				Vec2D vecLastPoint = vertices.get(getNumPoints()-1);
				PVector lastPoint = new PVector(vecLastPoint.x, vecLastPoint.y);
				for (int i=0; i<contours.size(); i++) {
					ArrayList<PVector> c = contours.get(i);
					PVector fp = c.get(0);
					PVector lp = c.get(c.size()-1);
					if (fp.dist(lastPoint) < distance) { 
						distance = fp.dist(lastPoint); 
						selectedContour = i; 
						selectedPoint = 0;
					}
					if (lp.dist(lastPoint) < distance) { 
						distance = lp.dist(lastPoint); 
						selectedContour = i; 
						selectedPoint = 1;
					}
				}
			} else {
				PVector closestPoint = new PVector(p.width, p.height);
				for (int i=0; i<contours.size(); i++) {
					ArrayList<PVector> c = contours.get(i);
					PVector fp = c.get(0);
					PVector lp = c.get(c.size()-1);
					if (fp.y > p.kinectHeight-5 && fp.x < closestPoint.x) { 
						closestPoint = fp; 
						selectedContour = i; 
						selectedPoint = 0;
					}
					if (lp.y > p.kinectHeight-5 && lp.x < closestPoint.y) { 
						closestPoint = lp; 
						selectedContour = i; 
						selectedPoint = 1;
					}
				}
			}

			// add contour to polygon
			ArrayList<PVector> contour = contours.get(selectedContour);
			if (selectedPoint > 0) { Collections.reverse(contour); }
			for (PVector pVector : contour) {
				add(new Vec2D(pVector.x, pVector.y));
			}
			contours.remove(selectedContour);
		}
	}

	// creates a shape-deflecting physics chain in the box2d world from this polygon
	void createBody() {
		// for stability the body is always created (and later destroyed)
		BodyDef bd = new BodyDef();
		body = p.box2d.createBody(bd);
		// if there are more than 0 points (aka a person on screen)...
		if (getNumPoints() > 0) {
			// create a vec2d array of vertices in box2d world coordinates from this polygon
			Vec2[] verts = new Vec2[getNumPoints() / 8];
			for (int i=0; i<getNumPoints() / 8; i++) {
				Vec2D v = vertices.get(i * 8);
				verts[i] = p.box2d.coordPixelsToWorld(v.x, v.y);
			}
			// create a chain from the array of vertices
			ChainShape chain = new ChainShape();
			chain.createChain(verts, verts.length);
			// create fixture in body from the chain (this makes it actually deflect other shapes)
			body.createFixture(chain, 1);
		}
	}

	// destroy the box2d body (important!)
	void destroyBody() {
		p.box2d.destroyBody(body);
	}
}
