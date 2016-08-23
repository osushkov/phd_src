
#include <iostream>
#include <cstdlib>
#include <GL/gl.h>
#include <GL/glut.h>

#include "Vector3D.h"
#include "Geometry.h"
#include "ArmForwardKinematics.h"
#include "ArmInverseKinematics.h"

#define CYLINDER_RESOLUTION 6
#define CYLINDER_WIDTH 1.0f

#define SPHERE_RESOLUTION 6
#define SPHERE_SIZE 2.0f

GLfloat LightAmbient[]=     { 0.5f, 0.5f, 0.5f, 1.0f };
GLfloat LightDiffuse[]=     { 1.0f, 1.0f, 1.0f, 1.0f };
GLfloat LightSpecular[]=    { 1.0f, 1.0f, 1.0f, 1.0f };
GLfloat LightPosition[]=    { 0.0f, 0.0f, -200.0f, 1.0f };

GLUquadricObj *quadratic;

Vector3D findTransformedPosition(Vector3D original, float pan, float tilt, float roll){
    original = original + Vector3D(0.0f, 0.0f, 10.0f);

    std::vector<Vector3D> rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), roll*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(0.0f, 3.5f, 2.7f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(1.0f, 0.0f, 0.0f), tilt*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(0.0f, 3.5f, 0.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), pan*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(0.0f, 8.9f, 0.0f);

    return original;
}

Vector3D findTransforedArmPosition(Vector3D original, std::vector<float> arm_joints){
    std::vector<Vector3D> rot_mat;

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), arm_joints[5]*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(0.0f, 0.0f, 7.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), arm_joints[4]*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(0.0f, 0.0f, 12.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), arm_joints[3]*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(-7.5f, 0.0f, 9.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), arm_joints[2]*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(0.0f, 0.0f, 21.0f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 1.0f, 0.0f), arm_joints[1]*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(0.0f, 0.0f, 12.5f);

    rot_mat = Geometry::axisRotationMatrix(Vector3D(0.0f, 0.0f, 1.0f), arm_joints[0]*M_PI/180.0f);
    original.matrixMultLeft(rot_mat);
    original = original + Vector3D(0.0f, 0.0f, 15.5f);

    return original;
}

void drawCamera(float pan, float tilt, float roll){
    glPushMatrix();
    glColor3f(1.0f, 1.0f, 1.0f);

    ///// J1
    glPushMatrix();
    glRotatef(-90.0f, 1.0f, 0.0, 0.0);
    gluCylinder(quadratic, CYLINDER_WIDTH, CYLINDER_WIDTH,
                8.9f, CYLINDER_RESOLUTION, CYLINDER_RESOLUTION);
    glPopMatrix();

    glTranslatef(0.0f, 8.9f, 0.0f);
    glRotatef(pan, 0.0f, 1.0f, 0.0f);
    gluSphere(quadratic, SPHERE_SIZE, SPHERE_RESOLUTION, SPHERE_RESOLUTION);

    ///// J2
    glPushMatrix();

    glPushMatrix();
    glRotatef(-90.0f, 1.0f, 0.0, 0.0);
    gluCylinder(quadratic, CYLINDER_WIDTH, CYLINDER_WIDTH,
                3.5f, CYLINDER_RESOLUTION, CYLINDER_RESOLUTION);
    glPopMatrix();

    glTranslatef(0.0f, 3.5f, 0.0f);
    glRotatef(tilt, 1.0f, 0.0f, 0.0f);
    gluSphere(quadratic, SPHERE_SIZE, SPHERE_RESOLUTION, SPHERE_RESOLUTION);

    ///// J3
    glPushMatrix();

    glPushMatrix();
    glRotatef(-90.0f, 1.0f, 0.0, 0.0);
    gluCylinder(quadratic, CYLINDER_WIDTH, CYLINDER_WIDTH,
                3.5f, CYLINDER_RESOLUTION, CYLINDER_RESOLUTION);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.0f, 3.5f, 0.0f);
    gluCylinder(quadratic, CYLINDER_WIDTH, CYLINDER_WIDTH,
                2.7f, CYLINDER_RESOLUTION, CYLINDER_RESOLUTION);
    glPopMatrix();

    glTranslatef(0.0f, 3.5f, 2.7f);
    glRotatef(roll, 0.0f, 0.0f, 1.0f);
    gluSphere(quadratic, SPHERE_SIZE, SPHERE_RESOLUTION, SPHERE_RESOLUTION);

    ///// Camera
    glPushMatrix();
    glPushMatrix();
    gluCylinder(quadratic, CYLINDER_WIDTH, CYLINDER_WIDTH,
                10.0f, CYLINDER_RESOLUTION, CYLINDER_RESOLUTION);
    glPopMatrix();

    glTranslatef(0.0f, 0.0f, 10.0f);
    gluSphere(quadratic, 0.7f, SPHERE_RESOLUTION, SPHERE_RESOLUTION);

    glPushMatrix();
    glTranslatef(-1.5f, 0.0f, 0.0f);
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    gluCylinder(quadratic, CYLINDER_WIDTH, CYLINDER_WIDTH,
                3.0f, CYLINDER_RESOLUTION, CYLINDER_RESOLUTION);
    glPopMatrix();

    glPopMatrix();

    glPopMatrix();
    glPopMatrix();
    glPopMatrix();
}


void drawArm(std::vector<float> joints){
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
    glPushMatrix();

    // J1
    glPushMatrix();

    glPushMatrix();
    gluCylinder(quadratic, CYLINDER_WIDTH, CYLINDER_WIDTH,
                15.5f, CYLINDER_RESOLUTION, CYLINDER_RESOLUTION);
    glPopMatrix();

    glTranslatef(0.0f, 0.0f, 15.5f);
    glRotatef(joints[0], 0.0f, 0.0f, 1.0f);
    glutSolidSphere(SPHERE_SIZE, SPHERE_RESOLUTION, SPHERE_RESOLUTION);

    // J2
    glPushMatrix();

    glPushMatrix();
    gluCylinder(quadratic, CYLINDER_WIDTH, CYLINDER_WIDTH,
                12.5f, CYLINDER_RESOLUTION, CYLINDER_RESOLUTION);
    glPopMatrix();

    glTranslatef(0.0f, 0.0f, 12.5f);
    glRotatef(joints[1], 0.0f, 1.0f, 0.0f);
    glutSolidSphere(SPHERE_SIZE, SPHERE_RESOLUTION, SPHERE_RESOLUTION);

    // J3
    glPushMatrix();

    glPushMatrix();
    gluCylinder(quadratic, CYLINDER_WIDTH, CYLINDER_WIDTH,
                21.0f, CYLINDER_RESOLUTION, CYLINDER_RESOLUTION);
    glPopMatrix();

    glTranslatef(0.0f, 0.0f, 21.0f);
    glRotatef(joints[2], 0.0f, 1.0f, 0.0f);
    glutSolidSphere(SPHERE_SIZE, SPHERE_RESOLUTION, SPHERE_RESOLUTION);

    // J4
    glPushMatrix();

    glPushMatrix();
    glRotatef(-90.0f, 0.0f, 1.0f, 0.0f);
    gluCylinder(quadratic, CYLINDER_WIDTH, CYLINDER_WIDTH,
                7.5f, CYLINDER_RESOLUTION, CYLINDER_RESOLUTION);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-7.5f, 0.0f, 0.0f);
    gluCylinder(quadratic, CYLINDER_WIDTH, CYLINDER_WIDTH,
                9.0f, CYLINDER_RESOLUTION, CYLINDER_RESOLUTION);
    glPopMatrix();

    glTranslatef(-7.5f, 0.0f, 9.0f);
    glRotatef(joints[3], 0.0f, 0.0f, 1.0f);
    glutSolidSphere(SPHERE_SIZE, SPHERE_RESOLUTION, SPHERE_RESOLUTION);

    // J5
    glPushMatrix();

    glPushMatrix();
    gluCylinder(quadratic, CYLINDER_WIDTH, CYLINDER_WIDTH,
                12.0f, CYLINDER_RESOLUTION, CYLINDER_RESOLUTION);
    glPopMatrix();

    glTranslatef(0.0f, 0.0f, 12.0f);
    glRotatef(joints[4], 0.0f, 1.0f, 0.0f);
    glutSolidSphere(SPHERE_SIZE, SPHERE_RESOLUTION, SPHERE_RESOLUTION);

    // J6
    glPushMatrix();

    glPushMatrix();
    gluCylinder(quadratic, CYLINDER_WIDTH, CYLINDER_WIDTH,
                7.0f, CYLINDER_RESOLUTION, CYLINDER_RESOLUTION);
    glPopMatrix();

    glTranslatef(0.0f, 0.0f, 7.0f);
    glRotatef(joints[5], 0.0f, 0.0f, 1.0f);
    glutSolidSphere(SPHERE_SIZE, SPHERE_RESOLUTION, SPHERE_RESOLUTION);

    glPopMatrix();
    glPopMatrix();
    glPopMatrix();
    glPopMatrix();
    glPopMatrix();
    glPopMatrix();

    glPopMatrix();
}


void init(void){
    glShadeModel(GL_FLAT);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);

    quadratic = gluNewQuadric();
    //gluQuadricNormals(quadratic, GLU_SMOOTH);
    //gluQuadricTexture(quadratic, GL_FALSE);

    glEnable(GL_NORMALIZE);
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDisable(GL_CULL_FACE);

}

void drawPoint(Vector3D pos){
    glPushMatrix();
    glColor3f(1.0f, 0.0f, 0.0f);

    glTranslatef(pos.x, pos.y, pos.z);
    gluSphere(quadratic, 0.7f, SPHERE_RESOLUTION, SPHERE_RESOLUTION);

    glPopMatrix();
}

void display(void){

    std::vector<float> joints;

    joints.push_back(0.0f);
    joints.push_back(0.0f);
    joints.push_back(90.0f);

    joints.push_back(0.0f);
    joints.push_back(0.0f);
    joints.push_back(90.0f);

    ArmForwardKinematics afk;
    std::vector<Vector3D> arm_pose = afk.getArmPoint(joints);

    std::vector<Vector3D> orientation;
    orientation.push_back(arm_pose[1]);
    orientation.push_back(arm_pose[2]);
    orientation.push_back(arm_pose[3]);

    std::vector<float> euler_angles = afk.gripperOrientationToEulerAngles(orientation);
    std::cout << euler_angles[0] << " " << euler_angles[1] << " " << euler_angles[2] << std::endl;

    std::cout << arm_pose[0].x << " " << arm_pose[0].y << " " << arm_pose[0].z << std::endl;


    ArmInverseKinematics ik;
    std::vector<float> rjoints = ik.getRequiredJoints(Vector3D(28.0f, 0.0f, 56.5f));
    for(unsigned i = 0; i < rjoints.size(); i++){
        std::cout << rjoints[i] << " ";
    }
    std::cout << std::endl;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glPushMatrix();
	gluLookAt(0.0, 50.0, 100.0, 0.0, 50.0, 0.0, 0.0, 1.0, 0.0);
	//glTranslatef(0.0f, -10.0f,-20.0f);

	glBegin(GL_QUADS);
        glNormal3f(0.0f, 1.0f, 0.0f);
        glVertex3f(-100.0f, 0.0f, -100.0f);
        glVertex3f(-100.0f, 0.0f, 100.0f);
        glVertex3f(100.0f, 0.0f, 100.0f);
        glVertex3f(100.0f, 0.0f, -100.0f);
	glEnd();
/*
	float pan = 0.0f;
	float tilt = 45.0f;
	float roll = 0.0f;

	drawCamera(pan, tilt, roll);

	Vector3D nx = findTransformedPosition(Vector3D(1.0f, 0.0f, 0.0f), pan, tilt, roll);
    Vector3D ny = findTransformedPosition(Vector3D(0.0f, 1.0f, 0.0f), pan, tilt, roll);
    Vector3D nz = findTransformedPosition(Vector3D(0.0f, 0.0f, 1.0f), pan, tilt, roll);

    drawPoint(nx);
    drawPoint(ny);
    drawPoint(nz);
*/
	std::vector<float> arm_joints = rjoints;

	glutSolidTeapot(10.0f);
	//drawArm(arm_joints);

	glPopMatrix();

    glutSwapBuffers ( );
}

void reshape(int w, int h){
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  if(h == 0){
     gluPerspective(60.0f, (float)w, 0.1, 200.0);
  }
  else{
     gluPerspective(60.0f, (float)w/(float)h, 0.1, 200.0 );
  }
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void keyboard ( unsigned char key, int x, int y){
    switch(key){
        case 27:
            exit(0);
            break;
        default:
            break;
    }
}

int main(int argc, char **argv){
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize(640, 480);
  glutCreateWindow("NeHe's OpenGL Framework");
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);

  init();
  glutMainLoop();

  return 0;
}

