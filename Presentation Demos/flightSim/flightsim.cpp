/*
 * The flightsim demo.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

#include "marballs.h"
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>
#include <cassert>

/**
 * The main demo class definition.
 */
class FlightSimDemo : public Application
{
    marballs::AeroControl left_wing;
    marballs::AeroControl right_wing;
    marballs::AeroControl rudder;
    marballs::Aero tail;
    marballs::RigidBody aircraft;
    marballs::ForceRegistry registry;

    marballs::Vector3 windspeed;

    float left_wing_control;
    float right_wing_control;
    float rudder_control;

    void resetPlane();

public:
    /** Creates a new demo object. */
    FlightSimDemo();
    virtual ~FlightSimDemo();

    /** Returns the window title for the demo. */
    virtual const char* getTitle();

    /** Display the particles. */
    virtual void display();

    /** Update the particle positions. */
    virtual void update();

    /** Handle a key press. */
    virtual void key(unsigned char key);
};

// Method definitions
FlightSimDemo::FlightSimDemo()
:
Application(),

right_wing(marballs::Matrix3(0,0,0, -1,-0.5f,0, 0,0,0),
           marballs::Matrix3(0,0,0, -0.995f,-0.5f,0, 0,0,0),
           marballs::Matrix3(0,0,0, -1.005f,-0.5f,0, 0,0,0),
           marballs::Vector3(-1.0f, 0.0f, 2.0f), &windspeed),

left_wing(marballs::Matrix3(0,0,0, -1,-0.5f,0, 0,0,0),
          marballs::Matrix3(0,0,0, -0.995f,-0.5f,0, 0,0,0),
          marballs::Matrix3(0,0,0, -1.005f,-0.5f,0, 0,0,0),
          marballs::Vector3(-1.0f, 0.0f, -2.0f), &windspeed),

rudder(marballs::Matrix3(0,0,0, 0,0,0, 0,0,0),
       marballs::Matrix3(0,0,0, 0,0,0, 0.01f,0,0),
       marballs::Matrix3(0,0,0, 0,0,0, -0.01f,0,0),
       marballs::Vector3(2.0f, 0.5f, 0), &windspeed),

tail(marballs::Matrix3(0,0,0, -1,-0.5f,0, 0,0,-0.1f),
     marballs::Vector3(2.0f, 0, 0), &windspeed),

left_wing_control(0), right_wing_control(0), rudder_control(0),

windspeed(0,0,0)
{
    // Set up the aircraft rigid body.
    resetPlane();

    aircraft.SetMass(2.5f);
    marballs::Matrix3 it;
    it.SetBlockInertiaTensor(marballs::Vector3(2,1,1), 1);
    aircraft.SetInertiaTensor(it);

    aircraft.SetDamping(0.8f, 0.8f);

    aircraft.SetAcceleration(marballs::Vector3::GRAVITY);
    aircraft.CalculateDerivedData();

    aircraft.SetAwake();
    aircraft.SetCanSleep(false);

    registry.Add(&aircraft, &left_wing);
    registry.Add(&aircraft, &right_wing);
    registry.Add(&aircraft, &rudder);
    registry.Add(&aircraft, &tail);
}

FlightSimDemo::~FlightSimDemo()
{
}

void FlightSimDemo::resetPlane()
{
    aircraft.SetPosition(0, 0, 0);
    aircraft.SetOrientation(1,0,0,0);

    aircraft.SetVelocity(0,0,0);
    aircraft.SetRotation(0,0,0);
}

static void drawAircraft()
{
    // Fuselage
    glPushMatrix();
    glTranslatef(-0.5f, 0, 0);
    glScalef(2.0f, 0.8f, 1.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Rear Fuselage
    glPushMatrix();
    glTranslatef(1.0f, 0.15f, 0);
    glScalef(2.75f, 0.5f, 0.5f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Wings
    glPushMatrix();
    glTranslatef(0, 0.3f, 0);
    glScalef(0.8f, 0.1f, 6.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Rudder
    glPushMatrix();
    glTranslatef(2.0f, 0.775f, 0);
    glScalef(0.75f, 1.15f, 0.1f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Tail-plane
    glPushMatrix();
    glTranslatef(1.9f, 0, 0);
    glScalef(0.85f, 0.1f, 2.0f);
    glutSolidCube(1.0f);
    glPopMatrix();
}

void FlightSimDemo::display()
{
    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    marballs::Vector3 pos = aircraft.GetPosition();
    marballs::Vector3 offset(4.0f+aircraft.GetVelocity().Magnitude(), 0, 0);
    offset = aircraft.GetTransform().TransformDirection(offset);
    gluLookAt(pos.x+offset.x, pos.y+5.0f, pos.z+offset.z,
              pos.x, pos.y, pos.z,
              0.0, 1.0, 0.0);

    glColor3f(0.6f,0.6f,0.6f);
    int bx = int(pos.x);
    int bz = int(pos.z);
    glBegin(GL_QUADS);
    for (int x = -20; x <= 20; x++) for (int z = -20; z <= 20; z++)
    {
        glVertex3f(bx+x-0.1f, 0, bz+z-0.1f);
        glVertex3f(bx+x-0.1f, 0, bz+z+0.1f);
        glVertex3f(bx+x+0.1f, 0, bz+z+0.1f);
        glVertex3f(bx+x+0.1f, 0, bz+z-0.1f);
    }
    glEnd();

    // Set the transform matrix for the aircraft
    marballs::Matrix4 transform = aircraft.GetTransform();
    GLfloat gl_transform[16];
    transform.fillGLArray(gl_transform);
    glPushMatrix();
    glMultMatrixf(gl_transform);

    // Draw the aircraft
    glColor3f(0,0,0);
    drawAircraft();
    glPopMatrix();

    glColor3f(0.8f, 0.8f, 0.8f);
    glPushMatrix();
    glTranslatef(0, -1.0f - pos.y, 0);
    glScalef(1.0f, 0.001f, 1.0f);
    glMultMatrixf(gl_transform);
    drawAircraft();
    glPopMatrix();

    char buffer[256];
    sprintf(
        buffer,
        "Altitude: %.1f | Speed %.1f",
        aircraft.GetPosition().y,
        aircraft.GetVelocity().Magnitude()
        );
    glColor3f(0,0,0);
    renderText(10.0f, 24.0f, buffer);

    sprintf(
        buffer,
        "Left Wing: %.1f | Right Wing: %.1f | Rudder %.1f",
        left_wing_control, right_wing_control, rudder_control
        );
    renderText(10.0f, 10.0f, buffer);
}

void FlightSimDemo::update()
{
    // Find the duration of the last frame in seconds
    float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
    if (duration <= 0.0f) return;

    // Start with no forces or acceleration.
    aircraft.ClearAccumulators();

    // Add the propeller force
    marballs::Vector3 propulsion(-10.0f, 0, 0);
    propulsion = aircraft.GetTransform().TransformDirection(propulsion);
    aircraft.AddForce(propulsion);

    // Add the forces acting on the aircraft.
    registry.UpdateForces(duration);

    // Update the aircraft's physics.
    aircraft.Integrate(duration);

    // Do a very basic collision detection and response with the ground.
    marballs::Vector3 pos = aircraft.GetPosition();
    if (pos.y < 0.0f)
    {
        pos.y = 0.0f;
        aircraft.SetPosition(pos);

        if (aircraft.GetVelocity().y < -10.0f)
        {
            resetPlane();
        }
    }

    Application::update();
}

const char* FlightSimDemo::getTitle()
{
    return "Marballs Flight Sim Demo";
}

void FlightSimDemo::key(unsigned char key)
{
    switch(key)
    {
    case 'q': case 'Q':
        rudder_control += 0.1f;
        break;

    case 'e': case 'E':
        rudder_control -= 0.1f;
        break;

    case 'w': case 'W':
        left_wing_control -= 0.1f;
        right_wing_control -= 0.1f;
        break;

    case 's': case 'S':
        left_wing_control += 0.1f;
        right_wing_control += 0.1f;
        break;

    case 'd': case 'D':
        left_wing_control -= 0.1f;
        right_wing_control += 0.1f;
        break;

    case 'a': case 'A':
        left_wing_control += 0.1f;
        right_wing_control -= 0.1f;
        break;

    case 'x': case 'X':
        left_wing_control = 0.0f;
        right_wing_control = 0.0f;
        rudder_control = 0.0f;
        break;

    case 'r': case 'R':
        resetPlane();
        break;

    default:
        Application::key(key);
    }

    // Make sure the controls are in range
    if (left_wing_control < -1.0f) left_wing_control = -1.0f;
    else if (left_wing_control > 1.0f) left_wing_control = 1.0f;
    if (right_wing_control < -1.0f) right_wing_control = -1.0f;
    else if (right_wing_control > 1.0f) right_wing_control = 1.0f;
    if (rudder_control < -1.0f) rudder_control = -1.0f;
    else if (rudder_control > 1.0f) rudder_control = 1.0f;

    // Update the control surfaces
    left_wing.SetControl(left_wing_control);
    right_wing.SetControl(right_wing_control);
    rudder.SetControl(rudder_control);
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
    return new FlightSimDemo();
}
