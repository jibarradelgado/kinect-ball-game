package com.jibarrad.hci.practical04;

import java.util.ArrayList;
import java.util.Collections;

import processing.core.*; //Processing core
import blobDetection.*; // blobs
import toxi.geom.*; // toxiclibs shapes and vectors

import org.jbox2d.collision.shapes.*; // jbox2d
import org.jbox2d.common.*; // jbox2d
import org.jbox2d.dynamics.*; // jbox2d

/**
 * 
 * @author Jorge
 * @author Amnon Owed http://www.creativeapplications.net/processing/kinect-physics-tutorial-for-processing/
 * 
 */
public class PolygonBlob extends Polygon2D {
	//to hold the box2d body
	Body body;
	
	BallGame p;
	
	public PolygonBlob(BallGame p) {
		this.p = p;
	}

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
						float dn = (float) Math.sqrt((fn.x*p.KINECT_WIDTH-eA.x*p.KINECT_WIDTH)
								*(fn.x*p.KINECT_WIDTH-eA.x*p.KINECT_WIDTH) 
								+(fn.y*p.KINECT_HEIGHT-eA.y*p.KINECT_HEIGHT)
								*(fn.y*p.KINECT_HEIGHT-eA.y*p.KINECT_HEIGHT));
						float dp = (float) Math.sqrt((fp.x*p.KINECT_WIDTH-eA.x*p.KINECT_WIDTH)
								*(fp.x*p.KINECT_WIDTH-eA.x*p.KINECT_WIDTH) 
								+(fp.y*p.KINECT_HEIGHT-eA.y*p.KINECT_HEIGHT)
								*(fp.y*p.KINECT_HEIGHT-eA.y*p.KINECT_HEIGHT));
						if (dn > 15 || dp > 15) {
							if (contour.size() > 0) {
								contour.add(new PVector(eB.x*p.KINECT_WIDTH, eB.y*p.KINECT_HEIGHT));
								contours.add(contour);
								contour = new ArrayList<PVector>();
							} else {
								contour.add(new PVector(eA.x*p.KINECT_WIDTH, eA.y*p.KINECT_HEIGHT));
							}
						} else {
							contour.add(new PVector(eA.x*p.KINECT_WIDTH, eA.y*p.KINECT_HEIGHT));
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
					if (fp.y > p.KINECT_HEIGHT-5 && fp.x < closestPoint.x) { 
						closestPoint = fp; 
						selectedContour = i; 
						selectedPoint = 0;
					}
					if (lp.y > p.KINECT_HEIGHT-5 && lp.x < closestPoint.y) { 
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
			Vec2[] verts = new Vec2[getNumPoints() / 4];
			for (int i=0; i<getNumPoints() / 4; i++) {
				Vec2D v = vertices.get(i * 4);
				verts[i] = p.box2d.coordPixelsToWorld(v.x, v.y);
			}
			// create a chain from the array of vertices
			ChainShape chain = new ChainShape();
			chain.createChain(verts, verts.length);
			// create fixture in body from the chain (this makes it actually deflect other shapes)
			body.createFixture(chain, 10);
		}
	}

	// destroy the box2d body (important!)
	void destroyBody() {
		p.box2d.destroyBody(body);
	}
}
