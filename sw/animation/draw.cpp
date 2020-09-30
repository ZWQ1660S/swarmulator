#include "draw.h"
#include "trigonometry.h"
#include <cmath>
#include "fitness_functions.h"

using namespace std;

void draw::set_color_white(const float intensity)
{
  glColor3ub(255 * intensity, 255 * intensity, 255 * intensity); // White
}

void draw::make_circle_coordinates(const float r, const int precision)
{
  glPushMatrix();
  float angle, x, y;
  for (int i = 0; i <= precision; i++) { // Resolution
    angle = 2 * M_PI * i / precision;
    y = (r * yrat) * cos(angle);
    x = (r * xrat) * sin(angle);
    glVertex2d(y, x);
  }
  glPopMatrix();
}

void draw::data()
{
  glPushMatrix();
  glRasterPos2f((-3.9 / zoom_scale - center_x), (-3.9 / zoom_scale - center_y));
  std::stringstream ss;
  this->set_color_white(1.);
  ss << "Time[s]:" << simtime_seconds << " \t" << "Fitness: " << evaluate_fitness();
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)ss.str().c_str());
  glPopMatrix();
}

void draw::axis_label()
{
  glPushMatrix();
  glRasterPos2f(3.9 / zoom_scale - center_x, 0.1 / zoom_scale);
  glutBitmapString(GLUT_BITMAP_8_BY_13,
                   (unsigned char *)std::string("E").c_str());
  glRasterPos2f(0.1 / zoom_scale, 3.9 / zoom_scale - center_y);
  glutBitmapString(GLUT_BITMAP_8_BY_13,
                   (unsigned char *)std::string("N").c_str());
  glPopMatrix();
}

void draw::agent_number(const uint16_t &ID)
{
  glPushMatrix();
  glRasterPos2f(-0.01, 0.035);
  set_color_white(1.0);
  std::stringstream ss;
  ss << (int)ID;
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)ss.str().c_str());
  glPopMatrix();
}

void draw::triangle(const float &scl, const float &orientation)
{
  glPushMatrix();
  glBegin(GL_POLYGON);
  glColor3ub(200, 000, 000); // Red
  glVertex2f(-1. * scl * yrat,  1. * scl * xrat);
  glVertex2f(-1. * scl * yrat, -1. * scl * xrat);
  glVertex2f(2. * scl * yrat,  0. * scl * xrat);
  glEnd();
  set_color_white(1.);
  glPopMatrix();
}

void draw::circle(const float &r)
{
  glPushMatrix();
  glBegin(GL_POLYGON);
  glColor3ub(200, 000, 000); // Red
  make_circle_coordinates(r, 10);
  glEnd();
  set_color_white(1.);
  glPopMatrix();
}

void draw::circle_loop(const float &r)
{
  glPushMatrix();
  glLineWidth(1);
  glBegin(GL_LINE_LOOP);
  make_circle_coordinates(r, 20);
  glEnd();
  set_color_white(1.);
  glPopMatrix();
}

void draw::line(const float &x, const float &y)
{
  glPushMatrix();
  glLineWidth(2.5);
  set_color_white(1.0);
  glBegin(GL_LINES);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(x * xrat, -y * yrat, 0);
  glEnd();
  glPopMatrix();
}

void draw::line(const float &x, const float &y, const float &width)
{
  glPushMatrix();
  glLineWidth(width);
  set_color_white(1.0);
  glBegin(GL_LINES);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(x * xrat, -y * yrat, 0);
  glEnd();
  glPopMatrix();
}

void draw::point()
{
  glPushMatrix();
  glPointSize(10.0);
  glBegin(GL_POINTS);
  glVertex3f(0, 0, 0);
  glEnd();
  glPopMatrix();
}

void draw::axes()
{
  glPushMatrix();

  glLineWidth(0.5);

  glBegin(GL_LINES);
  glLineStipple(1, 0xAAAA);  // [1]
  glEnable(GL_LINE_STIPPLE);
  set_color_white(0.5);
  glVertex3f(-1000., 0., 0.);
  glVertex3f(1000.0, 0., 0.);
  glEnd();

  glBegin(GL_LINES);
  glLineStipple(1, 0xAAAA);  // [1]
  glEnable(GL_LINE_STIPPLE);
  set_color_white(0.5);
  glVertex3f(0., -1000., 0.);
  glVertex3f(0., 1000., 0.);
  glEnd();

  glPopMatrix();

}

void draw::segment(const float &x0, const float &y0, const float &x1, const float &y1)
{
  glPushMatrix();
  glLineWidth(5);
  glBegin(GL_LINES);
  set_color_white(0.5);
  glVertex3f(y0 * xrat, x0 * yrat, 0.);
  glVertex3f(y1 * xrat, x1 * yrat, 0.);
  glEnd();
  glPopMatrix();
}

void draw::agent(const uint16_t &ID, const float x, const float y, const float &orientation)
{
  glPushMatrix();
  glTranslatef(y * xrat, x * yrat, 0.); // ENU to NED
  glRotatef(90.0 - rad2deg(orientation), 0., 0., 1);
  s[ID]->animation(); // Uses the animation function defined by the agent in use
  s[ID]->controller->animation(ID); // Draws additional stuff from the controller, such as sensors
  // agent_number(ID);
  glPopMatrix();
}

void draw::velocity_arrow(const uint16_t &ID, const float &x, const float &y, const float &v_x, const float &v_y)
{
  glPushMatrix();
  glTranslatef(y * xrat, x * yrat, 0.0); // ENU to NED
  glRotatef(90., 0., 0., 1.);
  line(v_x, v_y);
  glPopMatrix();
}

void draw::food(const float &x, const float &y)
{
  glPushMatrix();
  glTranslatef(y * xrat, x * yrat, 0.);
  glRotatef(90., 0., 0., 1.);
  point();
  glPopMatrix();
}
