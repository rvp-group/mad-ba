#include "gl_helpers.h"

using namespace Eigen;

namespace srrg2_qgl_viewport {

  class GLUWrapper {
  public:
    static GLUquadricObj* getQuadradic() {
      static GLUWrapper inst;
      return inst._quadratic;
    }
  protected:
    GLUWrapper() {
      _quadratic = gluNewQuadric();
      gluQuadricNormals(_quadratic, GLU_SMOOTH);
    }
    ~GLUWrapper() {
      gluDeleteQuadric(_quadratic);
    }
    GLUquadricObj *_quadratic;
  };


  void glGetColor3f(Vector3f& color_rgb_) {
    float c[4];
    glGetFloatv(GL_CURRENT_COLOR,c);
    color_rgb_.setZero();
    color_rgb_[0]=c[0];
    color_rgb_[1]=c[1];
    color_rgb_[2]=c[2];
  }

  void glGetColor4f(Vector4f& color_rgba_) {
    float c[4];
    glGetFloatv(GL_CURRENT_COLOR,c);
    color_rgba_.setZero();
    color_rgba_[0]=c[0];
    color_rgba_[1]=c[1];
    color_rgba_[2]=c[2];
    color_rgba_[3]=c[3];
  }


  void glAddColor3f(const Vector3f& color_rgb_){
    Vector3f current_rgb = Vector3f::Zero();
    glGetColor3f(current_rgb);

    current_rgb.noalias() += color_rgb_;

    glColor3f(current_rgb.x(), current_rgb.y(), current_rgb.z());
  }

  void glScaleColor3f(const Vector3f& scale_rgb_){
    Vector3f current_rgb = Vector3f::Zero();
    glGetColor3f(current_rgb);

    glColor3f(current_rgb.x() * scale_rgb_.x(),
              current_rgb.y() * scale_rgb_.y(),
              current_rgb.z() * scale_rgb_.y());
  }

  void glScaleColor4f(const Vector4f& scale_rgba_){
    Vector4f current_rgba = Vector4f::Zero();
    glGetColor4f(current_rgba);

    glColor4f(current_rgba[0] * scale_rgba_[0],
              current_rgba[1] * scale_rgba_[1],
              current_rgba[2] * scale_rgba_[2],
              current_rgba[3] * scale_rgba_[3]);
  }

  void drawReferenceSystem(const float& rf_linewidth_) {
    Vector3f current_rgb;
    glGetColor3f(current_rgb);

    float lw;
    glGetFloatv(GL_LINE_WIDTH, &lw);
    glPushAttrib(GL_COLOR);
    glColor3f(current_rgb.x()+0.3, current_rgb.y()+0.3, current_rgb.z()+0.3);
    drawBox(0.1, 0.1, 0.1);
    glLineWidth(rf_linewidth_);
    glBegin(GL_LINES);
    glColor3f(current_rgb.x()+.5,0,0);
    glNormal3f(0,0,1);
    glVertex3f(0,0,0);
    glVertex3f(1,0,0);
    glColor3f(0,current_rgb.y()+.5,0);
    glNormal3f(0,0,1);
    glVertex3f(0,0,0);
    glVertex3f(0,1,0);
    glColor3f(0,0,current_rgb.z()+.5);
    glNormal3f(1,1,0);
    glVertex3f(0,0,0);
    glVertex3f(0,0,1);
    glEnd();
    glPopAttrib();
    glLineWidth(lw);
  }

  void glMultMatrix(const Eigen::Isometry3f& iso) {
    Eigen::Matrix4f m = iso.matrix();
    m.row(3) << 0,0,0,1;
    glMultMatrixf(m.data());
  }

  void drawArrow2D(float len, float head_width, float head_len) {
    glBegin(GL_LINES);
    glVertex2f(0.f, 0.f);
    glVertex2f(len, 0.f);
    glEnd();

    glNormal3f(0.f,0.f,1.f);
    glBegin(GL_TRIANGLES);
    glVertex2f(len, 0.f);
    glVertex2f(len - head_len,  0.5f*head_width);
    glVertex2f(len - head_len, -0.5f*head_width);
    glEnd();
  }

  void drawPoseBox() {
    glPushMatrix();
    glScalef(0.5f,1.f,1.f);
    glPushMatrix();
    glScalef(1.f,0.25f,0.5f);
    glTranslatef(-0.5f,0.5f,0.f);
    glColor3f(1.0f, 0.3f, 0.3f);
    drawBox(1.f, 1.f, 1.f);
    glPopMatrix();

    glPushMatrix();
    glScalef(1.f,0.25f,0.5f);
    glTranslatef(-0.5f,-0.5f,0.f);
    glColor3f(1.0f, 0.1f, 0.1f);
    drawBox(1.f, 1.f, 1.f);
    glPopMatrix();

    glPushMatrix();
    glScalef(1.f,0.25f,0.5f);
    glTranslatef(+0.5f,0.5f,0.f);
    glColor3f(0.3f, 0.3f, 1.0f);
    drawBox(1.f, 1.f, 1.f);
    glPopMatrix();

    glPushMatrix();
    glScalef(1.f,0.25f,0.5f);
    glTranslatef(+0.5f,-0.5f,0.f);
    glColor3f(0.1f, 0.1f, 1.f);
    drawBox(1.f, 1.f, 1.f);
    glPopMatrix();
    glPopMatrix();
  }

  void drawBox(GLfloat l, GLfloat w, GLfloat h) {
    GLfloat sx = l*0.5f;
    GLfloat sy = w*0.5f;
    GLfloat sz = h*0.5f;

    glBegin(GL_QUADS);
    // bottom
    glNormal3f( 0.0f, 0.0f,-1.0f);
    glVertex3f(-sx, -sy, -sz);
    glVertex3f(-sx, sy, -sz);
    glVertex3f(sx, sy, -sz);
    glVertex3f(sx, -sy, -sz);
    // top
    glNormal3f( 0.0f, 0.0f,1.0f);
    glVertex3f(-sx, -sy, sz);
    glVertex3f(-sx, sy, sz);
    glVertex3f(sx, sy, sz);
    glVertex3f(sx, -sy, sz);
    // back
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(-sx, -sy, -sz);
    glVertex3f(-sx, sy, -sz);
    glVertex3f(-sx, sy, sz);
    glVertex3f(-sx, -sy, sz);
    // front
    glNormal3f( 1.0f, 0.0f, 0.0f);
    glVertex3f(sx, -sy, -sz);
    glVertex3f(sx, sy, -sz);
    glVertex3f(sx, sy, sz);
    glVertex3f(sx, -sy, sz);
    // left
    glNormal3f( 0.0f, -1.0f, 0.0f);
    glVertex3f(-sx, -sy, -sz);
    glVertex3f(sx, -sy, -sz);
    glVertex3f(sx, -sy, sz);
    glVertex3f(-sx, -sy, sz);
    //right
    glNormal3f( 0.0f, 1.0f, 0.0f);
    glVertex3f(-sx, sy, -sz);
    glVertex3f(sx, sy, -sz);
    glVertex3f(sx, sy, sz);
    glVertex3f(-sx, sy, sz);
    glEnd();
  }

  void drawBoxWireframe(GLfloat l, GLfloat w, GLfloat h) {
    GLfloat sx = l*0.5f;
    GLfloat sy = w*0.5f;
    GLfloat sz = h*0.5f;

    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    glBegin(GL_QUADS);
    // bottom
    glNormal3f( 0.0f, 0.0f,-1.0f);
    glVertex3f(-sx, -sy, -sz);
    glVertex3f(-sx, sy, -sz);
    glVertex3f(sx, sy, -sz);
    glVertex3f(sx, -sy, -sz);
    // top
    glNormal3f( 0.0f, 0.0f,1.0f);
    glVertex3f(-sx, -sy, sz);
    glVertex3f(-sx, sy, sz);
    glVertex3f(sx, sy, sz);
    glVertex3f(sx, -sy, sz);
    // back
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(-sx, -sy, -sz);
    glVertex3f(-sx, sy, -sz);
    glVertex3f(-sx, sy, sz);
    glVertex3f(-sx, -sy, sz);
    // front
    glNormal3f( 1.0f, 0.0f, 0.0f);
    glVertex3f(sx, -sy, -sz);
    glVertex3f(sx, sy, -sz);
    glVertex3f(sx, sy, sz);
    glVertex3f(sx, -sy, sz);
    // left
    glNormal3f( 0.0f, -1.0f, 0.0f);
    glVertex3f(-sx, -sy, -sz);
    glVertex3f(sx, -sy, -sz);
    glVertex3f(sx, -sy, sz);
    glVertex3f(-sx, -sy, sz);
    //right
    glNormal3f( 0.0f, 1.0f, 0.0f);
    glVertex3f(-sx, sy, -sz);
    glVertex3f(sx, sy, -sz);
    glVertex3f(sx, sy, sz);
    glVertex3f(-sx, sy, sz);
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

  }

  void drawPlane(GLfloat l, GLfloat w) {
    GLfloat sx = l*0.5f;
    GLfloat sy = w*0.5f;

    glBegin(GL_QUADS);
    glNormal3f( 0.0f, 0.0f, 1.0f);
    glVertex3f(-sx, -sy, 0.f);
    glVertex3f(-sx, sy, 0.f);
    glVertex3f(sx, sy, 0.f);
    glVertex3f(sx, -sy, 0.f);
    glEnd();
  }

  void drawSphere(GLfloat radius) {
    gluSphere(GLUWrapper::getQuadradic(), radius, 32, 32);
  }

  void drawEllipsoid(GLfloat r1, GLfloat r2, GLfloat r3) {
    GLboolean hasNormalization = glIsEnabled(GL_NORMALIZE);
    if (!hasNormalization)
      glEnable(GL_NORMALIZE);
    glPushMatrix();
    glScalef(r1, r2, r3);
    gluSphere(GLUWrapper::getQuadradic(), 1.0f, 32, 32);
    glPopMatrix();
    if (!hasNormalization)
      glDisable(GL_NORMALIZE);
  }

  void drawCone(GLfloat radius, GLfloat height) {
    glPushMatrix();
    glRotatef(-90.f, 1.f, 0.f, 0.f);
    glTranslatef(0.f, 0.f, - height/2.0f);
    gluCylinder(GLUWrapper::getQuadradic(), radius, 0.f, height, 32, 1);
    gluDisk(GLUWrapper::getQuadradic(), 0, radius, 32, 1);
    glPopMatrix();
  }

  void drawCylinder(GLfloat radius, GLfloat height) {
    glPushMatrix();
    glRotatef(-90, 1.f, 0.f, 0.f);
    glTranslatef(0.f, 0.f, + height/2.0f);
    gluDisk(GLUWrapper::getQuadradic(), 0.f, radius, 32, 1);
    glTranslatef(0, 0, - height);
    gluCylinder(GLUWrapper::getQuadradic(), radius, radius, height, 32, 1);
    glRotatef(180, 1.f, 0.f, 0.f);
    gluDisk(GLUWrapper::getQuadradic(), 0, radius, 32, 1);
    glPopMatrix();
  }

  void drawDisk(GLfloat radius) {
    glRotatef(90, 0.f, 1.f, 0.f);
    gluDisk(GLUWrapper::getQuadradic(), 0, radius, 32, 1);
  }

  void drawPyramid(GLfloat length, GLfloat height) {
    glPushMatrix();
    glTranslatef(0.f, 0.f, - height/2.0f);
    glRotatef(45, 0.f, 0.f, 1.f);
    gluCylinder(GLUWrapper::getQuadradic(), length, 0.f, height, 4, 1);
    gluDisk(GLUWrapper::getQuadradic(), 0, length, 4, 1);
    glPopMatrix();
  }


  void drawPyramidWireframe(float pyrH, float pyrW){
    glBegin(GL_LINE_LOOP);
    glNormal3f(0.0,0.0,-1.0);
    glVertex3f(pyrW,-pyrW,pyrH);
    glNormal3f(0.0,0.0,-1.0);
    glVertex3f(pyrW,pyrW,pyrH);
    glNormal3f(0.0,0.0,-1.0);
    glVertex3f(-pyrW,pyrW,pyrH);
    glNormal3f(0.0,0.0,-1.0);
    glVertex3f(-pyrW,-pyrW,pyrH);
    //glNormal3f(0.0,0.0,-1.0);
    glEnd();
    //draw the nose
    glBegin(GL_LINES);

    glVertex3f(pyrW,-pyrW,pyrH);
    //glColor3f(pyrH,0.0,0.0);
    glVertex3f(0.0,0.0,0.0);
    glNormal3f(0.0,0.0,-1.0);

    //glColor3f(pyrH,pyrH,pyrH);
    glVertex3f(pyrW,pyrW,pyrH);
    //glColor3f(pyrH,0.0,0.0);
    glVertex3f(0.0,0.0,0.0);
    glNormal3f(0.0,0.0,-1.0);

    //glColor3f(pyrH,pyrH,pyrH);
    glVertex3f(-pyrW,pyrW,pyrH);
    //glColor3f(pyrH,0.0,0.0);
    glVertex3f(0.0,0.0,0.0);
    glNormal3f(0.0,0.0,-1.0);

    //glColor3f(pyrH,pyrH,pyrH);
    glVertex3f(-pyrW,-pyrW,pyrH);
    // glColor3f(pyrH,0.0,0.0);
    glVertex3f(0.0,0.0,0.0);
    glNormal3f(0.0,0.0,-1.0);
    glEnd();
  }

  void drawRangeRing(GLfloat range, GLfloat fov, GLfloat range_width) {
    glPushMatrix();
    glRotatef((fov/2.0f) - 90, 0.f, 0.f, 1.f);
    gluPartialDisk(GLUWrapper::getQuadradic(), range, range + range_width, 32, 1, 0.f, fov);
    glPopMatrix();
  }

  void drawSlice(GLfloat radius, GLfloat height, GLfloat fov, int slices_per_circle) {
    double fov_rad = fov/180.*M_PI;
    int num_slices = int(slices_per_circle * (fov_rad / (2*M_PI))) + 1;
    double angle_step = fov_rad / num_slices;
    double angle_step_half = angle_step * 0.5;

    GLfloat height_half = height * 0.5f;
    GLfloat lower_z = -height_half;
    GLfloat upper_z =  height_half;

    GLfloat last_x = float(std::cos(-fov_rad * 0.5f) * radius);
    GLfloat last_y = float(std::sin(-fov_rad * 0.5f) * radius);

    glPushMatrix();
    glBegin(GL_TRIANGLES);
    glNormal3f((float)std::sin(-fov_rad * 0.5), (float)-std::cos(-fov_rad * 0.5), 0.f);
    glVertex3f(0.f, 0.f, upper_z);
    glVertex3f(0.f, 0.f, lower_z);
    glVertex3f(last_x, last_y, upper_z);
    glVertex3f(last_x, last_y, upper_z);
    glVertex3f(last_x, last_y, lower_z);
    glVertex3f(0.f, 0.f, lower_z);

    double start_angle = -0.5*fov_rad + angle_step;
    double angle       = start_angle;
    for (int i = 0; i < num_slices; ++i) {
      GLfloat x = float(std::cos(angle) * radius);
      GLfloat y = float(std::sin(angle) * radius);
      GLfloat front_normal_x = (float)std::cos(angle + angle_step_half);
      GLfloat front_normal_y = (float)std::sin(angle + angle_step_half);

      // lower triangle
      glNormal3f(0.f, 0.f, -1.f);
      glVertex3f(0.f, 0.f, lower_z);
      glVertex3f(x, y, lower_z);
      glVertex3f(last_x, last_y, lower_z);
      // upper
      glNormal3f(0.f, 0.f, 1.f);
      glVertex3f(0.f, 0.f, upper_z);
      glVertex3f(x, y, upper_z);
      glVertex3f(last_x, last_y, upper_z);
      //front rectangle (we use two triangles)
      glNormal3f(front_normal_x, front_normal_y, 0.f);
      glVertex3f(last_x, last_y, upper_z);
      glVertex3f(last_x, last_y, lower_z);
      glVertex3f(x, y, upper_z);
      glVertex3f(x, y, upper_z);
      glVertex3f(x, y, lower_z);
      glVertex3f(last_x, last_y, lower_z);

      last_x = x;
      last_y = y;
      angle += angle_step;
    }

    glNormal3f(float(-std::sin(fov_rad * 0.5)), float(std::cos(fov_rad * 0.5)), -0.f);
    glVertex3f(0.f, 0.f, upper_z);
    glVertex3f(0.f, 0.f, lower_z);
    glVertex3f(last_x, last_y, upper_z);
    glVertex3f(last_x, last_y, upper_z);
    glVertex3f(last_x, last_y, lower_z);
    glVertex3f(0.f, 0.f, lower_z);

    glEnd();
    glPopMatrix();
  }

  void drawPoint(float pointSize){
    glPointSize(pointSize);
    glBegin(GL_POINTS);
    glVertex3f(0,0,0);
    glEnd();
  }

  void drawText(const char* text){
    size_t i = 0;
    glPushMatrix();
    glRasterPos2i(0,0);
    while(text[i] != '\0')
      glutBitmapCharacter(GLUT_BITMAP_9_BY_15, (int)text[i++]);
    glPopMatrix();
    glNormal3f(0.0, 0.0, -1.f);
      //glutStrokeCharacter(GLUT_STROKE_ROMAN, text[i++]);
  }

} //ia end namespace
