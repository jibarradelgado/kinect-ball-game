package com.jibarrad.hci.practical04;

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
public class BallShape {

	BallGame p;
	// usually one would probably make a generic Shape class and subclass different types (circle, polygon), but that
	// would mean at least 3 instead of 1 class, so for this tutorial it's a combi-class CustomShape for all types of shapes
	// to save some space and keep the code as concise as possible I took a few shortcuts to prevent repeating the same code
	// to hold the box2d body
	Body body;
	// to hold the Toxiclibs polygon shape
	Polygon2D toxiPoly;
	// custom color for each shape
	int col;
	// radius (also used to distinguish between circles and polygons in this combi-class
	float r;

	public BallShape(BallGame p,float x, float y, float r) {
		this.p = p;
		this.r = r;
		// create a body (polygon or circle based on the r)
		makeBody(x, y);
		// get a random color
		col = p.getRandomColor();
	}

	void makeBody(float x, float y) {
		// define a dynamic body positioned at xy in box2d world coordinates,
		// create it and set the initial values for this box2d body's speed and angle
		BodyDef bd = new BodyDef();
		bd.type = BodyType.DYNAMIC;
		bd.position.set(p.box2d.coordPixelsToWorld(new Vec2(x, y)));
		body = p.box2d.createBody(bd);
		body.setLinearVelocity(new Vec2(p.generateRandom(-8, 8), p.generateRandom(2,  8)));
		body.setAngularVelocity(p.generateRandom(-5, 5));

		// depending on the r this combi-code creates either a box2d polygon or a circle
		if (r == -1) {
			// box2d polygon shape
			PolygonShape sd = new PolygonShape();
			// toxiclibs polygon creator (triangle, square, etc)
			toxiPoly = new Circle(p.generateRandom(5, 20)).toPolygon2D(p.generateRandom(3, 6));
			// place the toxiclibs polygon's vertices into a vec2d array
			Vec2[] vertices = new Vec2[toxiPoly.getNumPoints()];
			for (int i=0; i<vertices.length; i++) {
				Vec2D v = toxiPoly.vertices.get(i);
				vertices[i] = p.box2d.vectorPixelsToWorld(new Vec2(v.x, v.y));
			}
			// put the vertices into the box2d shape
			sd.set(vertices, vertices.length);
			// create the fixture from the shape (deflect things based on the actual polygon shape)
			body.createFixture(sd, 1);
		} else {
			// box2d circle shape of radius r
			CircleShape cs = new CircleShape();
			cs.m_radius = p.box2d.scalarPixelsToWorld(r);
			// tweak the circle's fixture def a little bit
			FixtureDef fd = new FixtureDef();
			fd.shape = cs;
			fd.density = 1;
			fd.friction = (float)0.01;
			fd.restitution = (float)0.3;
			// create the fixture from the shape's fixture def (deflect things based on the actual circle shape)
			body.createFixture(fd);
		}
	}

	// method to loosely move shapes outside a person's polygon
	// (alternatively you could allow or remove shapes inside a person's polygon)
	void update() {
		// get the screen position from this shape (circle of polygon)
		Vec2 posScreen = p.box2d.getBodyPixelCoord(body);
		// turn it into a toxiclibs Vec2D
		Vec2D toxiScreen = new Vec2D(posScreen.x, posScreen.y);
		// check if this shape's position is inside the person's polygon
		boolean inBody = p.poly.containsPoint(toxiScreen);
		// if a shape is inside the person
		if (inBody) {
			// find the closest point on the polygon to the current position
			Vec2D closestPoint = toxiScreen;
			float closestDistance = 9999999;
			for (Vec2D v : p.poly.vertices) {
				float distance = v.distanceTo(toxiScreen);
				if (distance < closestDistance) {
					closestDistance = distance;
					closestPoint = v;
				}
			}
			// create a box2d position from the closest point on the polygon
			Vec2 contourPos = new Vec2(closestPoint.x, closestPoint.y);
			Vec2 posWorld = p.box2d.coordPixelsToWorld(contourPos);
			float angle = body.getAngle();
			// set the box2d body's position of this CustomShape to the new position (use the current angle)
			body.setTransform(posWorld, angle);
		}
	}

	// display the customShape
	void display() {
		// get the pixel coordinates of the body
		Vec2 pos = p.box2d.getBodyPixelCoord(body);
		p.pushMatrix();
		// translate to the position
		p.translate(pos.x, pos.y);
		p.noStroke();
		// use the shape's custom color
		p.fill(col);
		// depending on the r this combi-code displays either a polygon or a circle
		if (r == -1) {
			// rotate by the body's angle
			float a = body.getAngle();
			p.rotate(-a); // minus!
			p.gfx.polygon2D(toxiPoly);
		} else {
			p.ellipse(0, 0, r*2, r*2);
		}
		p.popMatrix();
	}

	/**
	 * 
	 * @return true if the ball is offscreen. 
	 */
	boolean ballRules() {
		Vec2 posScreen = p.box2d.getBodyPixelCoord(body);
		
		//Bouncing the ball at a low point of the screen
		if (posScreen.y > p.height - 325) {
			body.m_linearVelocity.y = -(body.m_linearVelocity.y);
		} 
		
		//At certain point of the ceiling the ball
		//will start accelerating
		if (posScreen.y > 100) {
			body.m_linearVelocity.x *= 1.09f;
		}
		
		boolean offscreen = false;
		
		//Determine the scores when the ball goes to an extreme x position
		if (posScreen.x < 0 || posScreen.x > p.width) {
			if (posScreen.x < 0) {
				p.setScoreRight(p.getScoreRight() + 1);
			}
			if (posScreen.x > p.width) {
				p.setScoreLeft(p.getScoreLeft() + 1);
			}
			offscreen = true;
		}
		
		if (offscreen) {
			p.box2d.destroyBody(body);
			return true;
		}
		
		return false;
	}
	

}
