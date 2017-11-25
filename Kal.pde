
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.collision.shapes.*;

import shiffman.box2d.Box2DProcessing;

Box2DProcessing box2d;
PGraphics ref;
float gravityMod = 1;
float velocityMod = 1;

color backgroundColor;

class Box {
  
  Body body;
  float w, h;
  color sc, fc;
  int shapeType;
  
  Box() {
    w = random(6, 15);
    h = random(2, 5);
    
    BodyDef bd = new BodyDef();
    Vec2 center = box2d.coordPixelsToWorld(mouseX, mouseY);
    bd.type = BodyType.DYNAMIC;
    bd.linearDamping = 0.0000001;
    bd.angularDamping = 0.0000001;
    bd.position.set(center);
    body = box2d.createBody(bd);
    
    PolygonShape ps = new PolygonShape();
    float box2Dw = box2d.scalarPixelsToWorld(w / 2);
    float box2Dh = box2d.scalarPixelsToWorld(h / 2);
    ps.setAsBox(box2Dw, box2Dh);
    
    FixtureDef fd = new FixtureDef();
    fd.shape = ps;
    fd.density = 1;
    fd.friction = 0.03;
    fd.restitution = 0.6;
    
    body.createFixture(fd);
    body.setGravityScale(50 * gravityMod);
    
    colorMode(HSB, 360, 100, 100);
    float randomHue = random(1, 360);
    sc = color(randomHue, 80, 100);
    fc = color(randomHue, 80, 80, 230);
    
     float r = random(0, 1);
     if (r < 0.5)  shapeType = 0;
     else if (r < 0.75) shapeType = 1;
     // else if (r < 0.75) shapeType = 2;
     else shapeType = 2;
     // shapeType = 2;
  }
  
  void drawShape() {
    Vec2 pos = box2d.getBodyPixelCoord(body);
    float a = body.getAngle();
    switch(shapeType) {
      case 0:
        ref.pushMatrix();
          ref.translate(pos.x, pos.y);
          ref.rotate(-a);
          ref.fill(fc);
          ref.stroke(sc);
          ref.rectMode(CENTER);
          ref.rect(0, 0, 1, 1);
        ref.popMatrix();
        break;
      case 1:
        ref.pushMatrix();
          ref.translate(pos.x, pos.y);
          ref.rotate(-a);
          ref.fill(fc);
          ref.stroke(sc);
          ref.rectMode(CENTER);
          ref.triangle(- w / 2, 0, w / 2, - h / 2, w / 2, h / 2);
        ref.popMatrix();
        break;
      case 2:
        ref.pushMatrix();
          ref.translate(pos.x, pos.y);
          ref.rotate(-a);
          ref.fill(fc);
          ref.stroke(sc);
          ref.rectMode(CENTER);
          ref.quad(- w / 2, 0, w / 4, - h / 2, w / 2, 0, w / 4, h / 2);
        ref.popMatrix();
        break;
      case 3:
        ref.pushMatrix();
          ref.translate(pos.x, pos.y);
          ref.rotate(-a);
          ref.fill(fc);
          ref.stroke(sc);
          ref.rectMode(CENTER);
          ref.ellipse(0, 0, w, h);
        ref.popMatrix();
        break;
    }
  }
  
  void display() {
    drawShape();
  }
  
  void killBody() {
    box2d.destroyBody(body);
  }
}

class BoundingTriangle {
  float cx, cy;
  float l;
  color col;
  
  Body body, centerBody;
  RevoluteJointDef rjd;
  RevoluteJoint joint;
  
  BoundingTriangle(float cx_, float cy_, float l_) {
    cx = cx_;
    cy = cy_;
    l = l_;
    
    BodyDef bd = new BodyDef();
    bd.position.set(box2d.coordPixelsToWorld(cx, cy));
    bd.type = BodyType.KINEMATIC;
    body = box2d.createBody(bd);
    
    Vec2[] vertices = new Vec2[3];
    vertices[0] = box2d.coordPixelsToWorld(cx - l / 2, cy + l * sqrt(3) / 2 - l / sqrt(3));
    vertices[1] = box2d.coordPixelsToWorld(cx, cy - l / sqrt(3));
    vertices[2] = box2d.coordPixelsToWorld(cx + l / 2, cy + l * sqrt(3) / 2 - l / sqrt(3));
    
    EdgeShape a = new EdgeShape(), b = new EdgeShape(), c = new EdgeShape();
    a.set(vertices[0], vertices[1]);
    b.set(vertices[1], vertices[2]);
    c.set(vertices[2], vertices[0]);
    body.createFixture(a, 1.0);
    body.createFixture(b, 1.0);
    body.createFixture(c, 1.0);
    body.setAngularVelocity(velocityMod * 3 * PI / 4);
  }
  
  void display() {
    float a = body.getAngle();
    ref.fill(backgroundColor);
    ref.noStroke();
    ref.pushMatrix();
      ref.translate(cx, cy);
      ref.rotate(-a);
      ref.triangle(- l / 2, l * sqrt(3) / 2 - l / sqrt(3), 0,  - l / sqrt(3), l / 2, l * sqrt(3) / 2 - l / sqrt(3));
    ref.popMatrix();
  }
}

ArrayList<Box> boxes;
BoundingTriangle t;

void setup() {
  size(600, 600, P3D);
  colorMode(HSB, 360, 100, 100, 220);
  backgroundColor = color(170, 90, 30);
  background(backgroundColor);
  smooth();
  pixelDensity(2);
  box2d = new Box2DProcessing(this);
  box2d.createWorld();
  boxes = new ArrayList<Box>();
  t = new BoundingTriangle(300, 300, 150);
  ref = createGraphics(600, 600);
  calculateGrid();
}

PImage refImage;

void draw() {
  ref.beginDraw();
  ref.background(backgroundColor);
  ref.noStroke();
  t.display();
  if(mousePressed) {
    Box p = new Box();
    boxes.add(p);
  }
  for(Box b: boxes) {
    b.display();
  }
  ref.endDraw();
  pushMatrix();
  imageMode(CENTER);
  translate(300, 300);
  rotateZ(t.body.getAngle());
  image(ref, 0, 0);
  popMatrix();
  //stroke(0);
  //noFill();
  //rect(225, 213, 150, 130);
  refImage = get(225 * 2, 213 * 2, 150 * 2, 130 * 2);
  drawHexGrid();
  pointLight(300, 300, 50, 255, 255, 255);
  box2d.step();
  if (isRecording) {
    saveFrame("frames/f#####.png");
  }
}

boolean isRecording = false;

void randomizeBackgroundColor() {
  colorMode(HSB, 360, 100, 100);
  backgroundColor = color(random(0, 360), 90, 30);
}

void incrGravity() {
  gravityMod += 0.1;
  for(Box b: boxes) {
    b.body.setGravityScale(50 * gravityMod);
  }
}

void incrVelocity() {
  velocityMod += 0.1;
  t.body.setAngularVelocity(velocityMod * 3 * PI / 4);
}

void keyPressed()
{
  if (key == ' ') isRecording = !isRecording;
  if (key == 'b') randomizeBackgroundColor();
  if (key == 'g') incrGravity();
  if (key == 'v') incrVelocity();
}

float angleStep = radians(60);
float radius = 150;
float uv0 = 0, uv1 = 1;
float gridWidth, gridHeight;

void calculateGrid()
{
  float angle = radians(360/6/2);
  float b = radius * cos(angle);
  gridWidth = b * 2;
  float a = sqrt(radius * radius - b*b);
  gridHeight = radius + a;
}

void drawHexGrid()
{ 
  int xCount = int(width/gridWidth) + 2;
  int yCount = int(height/gridHeight) + 2;
  float xOffset;

  for (int x = -2; x < xCount; x++)
  {
    for (int y = -2; y < yCount; y++)
    {
      if (y%2 == 0) xOffset = 0;
      else xOffset = gridWidth/2;

      pushMatrix();
      translate(x*gridWidth+xOffset, y*gridHeight, 0);
      rotateZ(angleStep/2);

      drawHex();

      popMatrix();
    }
  }
}

void drawHex() {
  pushMatrix();
  noStroke();
  translate(300, 300 - 150 / sqrt(3));
  beginShape(TRIANGLE_FAN);
  texture(refImage);
  textureMode(NORMAL);
  vertex(0, 0, 0, 0.5, 0);
  for (int i = 0; i < 6; i++)
  {
    float x1 = cos(angleStep * i) * radius;
    float y1 = sin(angleStep * i) * radius;
    float x2 = cos(angleStep * (i + 1)) * radius;
    float y2 = sin(angleStep * (i + 1)) * radius;

    if (i%2 == 0)
    {
      vertex(x1, y1, 0, uv0, 1);
      vertex(x2, y2, 0, uv1, 1);
    } else
    {
      vertex(x1, y1, 0, uv1, 1);
      vertex(x2, y2, 0, uv0, 1);
    }
  }
  endShape();
  popMatrix();
}