////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <cstddef>
#include <vector>
#include <list>
#include <string>
#include <memory>
#include <map>
#include <fstream>
#include <stdexcept>
#include <math.h>
#if __GNUG__
#   include <tr1/memory>
#endif

#ifdef __MAC__
#   include <OpenGL/gl3.h>
#   include <GLUT/glut.h>
#else
#   include <GL/glew.h>
#   include <GL/glut.h>
#endif

#include "ppm.h"
#include "cvec.h"
#include "quat.h"
#include "matrix4.h"
#include "rigtform.h"
#include "glsupport.h"
#include "geometrymaker.h"
#include "arcball.h"
#include "scenegraph.h"
#include "sgutils.h"
#include "cube.h"
#include "mesh.h"
#include "metaballs.h"
#include "asstcommon.h"
#include "drawer.h"
#include "picker.h"
#include "geometry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef EPSILON
#define EPSILON 0.01f
#endif

using namespace std;
using namespace tr1;

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.5.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.5. Use GLSL 1.5 unless your system does not support it.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------
const bool g_Gl2Compatible = false;


static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -4.5;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

enum SkyMode {WORLD_SKY=0, SKY_SKY=1};

static int g_windowWidth = 900;
static int g_windowHeight = 700;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static bool g_spaceDown = false;         // space state, for middle mouse emulation
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;

static SkyMode g_activeCameraFrame = WORLD_SKY;

static bool g_displayArcball = false;
static double g_arcballScreenRadius = 100; // number of pixels
static double g_arcballScale = 1;

static bool g_pickingMode = false;

static bool g_playingAnimation = false;

// new Materials system
static shared_ptr<Material> g_redDiffuseMat,
                            g_blueDiffuseMat,
                            g_balltexMat,
                            g_bumpFloorMat,
                            g_arcballMat,
                            g_pickingMat,
                            g_lightMat,
                            g_lampLightMat,
                            g_glassMat;

shared_ptr<Material> g_overridingMaterial;

// --------- Geometry
typedef SgGeometryShapeNode MyShapeNode;

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere, g_ellipsoid;

////////// metaball things

static float g_timer = 0.0;
static const int g_minGridSize = 10;
static int g_gridSize = 60;
static double g_threshold = 20;

static CubeGrid g_cubeGrid;
static shared_ptr<SimpleGeometryPNC> g_cubeGridGeometry;
static shared_ptr<SgRbtNode> g_cubeGridNode;

static int g_numMetaballs = 5;
static double g_radiusList [5] = {1.8, 2.7, 2.6, 3.1, 3.0};

static vector<Metaball> g_metaballs;
static vector<Cvec3> g_metaballPos;
static int g_animationSpeed = 70;

///////// end of that

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node, g_light1, g_light2;
static shared_ptr<SgRbtNode> g_lamp;

static shared_ptr<SgRbtNode> g_currentCameraNode;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode;
static bool g_smoothSubdRendering = false;

class Animator {
public:
  typedef vector<shared_ptr<SgRbtNode> > SgRbtNodes;
  typedef vector<RigTForm> KeyFrame;
  typedef list<KeyFrame> KeyFrames;
  typedef KeyFrames::iterator KeyFrameIter;

private:
  SgRbtNodes nodes_;
  KeyFrames keyFrames_;

public:
  void attachSceneGraph(shared_ptr<SgNode> root) {
    nodes_.clear();
    keyFrames_.clear();
    dumpSgRbtNodes(root, nodes_);
  }

  void loadAnimation(const char *filename) {
    ifstream f(filename, ios::binary);
    if (!f)
      throw runtime_error(string("Cannot load ") + filename);
    int numFrames, numRbtsPerFrame;
    f >> numFrames >> numRbtsPerFrame;
    if (numRbtsPerFrame != nodes_.size()) {
      cerr << "Number of Rbt per frame in " << filename
           <<" does not match number of SgRbtNodes in the current scene graph.";
      return;
    }

    Cvec3 t;
    Quat r;
    keyFrames_.clear();
    for (int i = 0; i < numFrames; ++i) {
      keyFrames_.push_back(KeyFrame());
      keyFrames_.back().reserve(numRbtsPerFrame);
      for (int j = 0; j < numRbtsPerFrame; ++j) {
        f >> t[0] >> t[1] >> t[2] >> r[0] >> r[1] >> r[2] >> r[3];
        keyFrames_.back().push_back(RigTForm(t, r));
      }
    }
  }

  void saveAnimation(const char *filename) {
    ofstream f(filename, ios::binary);
    int numRbtsPerFrame = nodes_.size();
    f << getNumKeyFrames() << ' ' << numRbtsPerFrame << '\n';
    for (KeyFrames::const_iterator frameIter = keyFrames_.begin(), e = keyFrames_.end(); frameIter != e; ++frameIter) {
      for (int j = 0; j < numRbtsPerFrame; ++j) {
        const RigTForm& rbt = (*frameIter)[j];
        const Cvec3& t = rbt.getTranslation();
        const Quat& r = rbt.getRotation();
        f << t[0] << ' ' << t[1] << ' ' << t[2] << ' '
        << r[0] << ' ' << r[1] << ' ' << r[2] << ' ' << r[3] << '\n';
      }
    }
  }

  int getNumKeyFrames() const {
    return keyFrames_.size();
  }

  int getNumRbtNodes() const {
    return nodes_.size();
  }

  void animate(double t) {
  if (t < 0 || t > keyFrames_.size() - 3)
    throw runtime_error("Invalid animation time parameter. Must be in the range [0, numKeyFrames - 3]");

  t += 1; // interpret the key frames to be at t= -1, 0, 1, 2, ...
  const int integralT = int(floor(t));
  const double fraction = t - integralT;

  KeyFrameIter f1 = getNthKeyFrame(integralT), f0 = f1, f2 = f1;
  --f0;
  ++f2;
  KeyFrameIter f3 = f2;
  ++f3;
  if (f3 == keyFrames_.end()) // this might be true when t is exactly keyFrames_.size()-3.
    f3 = f2; // in which case we step back

  for (int i = 0, n = nodes_.size(); i < n; ++i) {
    nodes_[i]->setRbt(interpolateCatmullRom((*f0)[i], (*f1)[i], (*f2)[i], (*f3)[i], fraction));
  }
}

  KeyFrameIter keyFramesBegin() {
    return keyFrames_.begin();
  }

  KeyFrameIter keyFramesEnd() {
    return keyFrames_.end();
  }

  KeyFrameIter getNthKeyFrame(int n) {
    KeyFrameIter frameIter = keyFrames_.begin();
    advance(frameIter, n);
    return frameIter;
  }

  void deleteKeyFrame(KeyFrameIter keyFrameIter) {
    keyFrames_.erase(keyFrameIter);
  }

  void pullKeyFrameFromSg(KeyFrameIter keyFrameIter) {
    for (int i = 0, n = nodes_.size(); i < n; ++i) {
      (*keyFrameIter)[i] = nodes_[i]->getRbt();
    }
  }

  void pushKeyFrameToSg(KeyFrameIter keyFrameIter) {
    for (int i = 0, n = nodes_.size(); i < n; ++i) {
      nodes_[i]->setRbt((*keyFrameIter)[i]);
    }
  }

  KeyFrameIter insertEmptyKeyFrameAfter(KeyFrameIter beforeFrame) {
    if (beforeFrame != keyFrames_.end())
      ++beforeFrame;

    KeyFrameIter frameIter = keyFrames_.insert(beforeFrame, KeyFrame());
    frameIter->resize(nodes_.size());
    return frameIter;
  }

};
/*
static int g_msBetweenKeyFrames = 2000; // 2 seconds between keyframes
static int g_animateFramesPerSecond = 60; // frames to render per second during animation playback


static Animator g_animator;
static Animator::KeyFrameIter g_curKeyFrame;
static int g_curKeyFrameNum;
*/
///////////////// END OF G L O B A L S //////////////////////////////////////////////////

static void initGround() {
  int ibLen, vbLen;
  getPlaneVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makePlane(g_groundSize*2, vtx.begin(), idx.begin());
  g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCubes() {
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
  int ibLen, vbLen;
  getSphereVbIbLen(20, 10, vbLen, ibLen);

  // Temporary storage for sphere Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);
  makeSphere(1, 20, 10, vtx.begin(), idx.begin());
  g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

static void initEllipsoid() {
  int ibLen, vbLen;
  getSphereVbIbLen(20, 10, vbLen, ibLen);

  // Temporary storage for sphere Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);
  makeEllipsoid(1, 20, 10, vtx.begin(), idx.begin());
  g_ellipsoid.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

static VertexPN getVertexPN(Mesh& m, const int face, const int vertex) {
  const Mesh::Face f = m.getFace(face);
  const Cvec3 n = g_smoothSubdRendering ? f.getVertex(vertex).getNormal() : f.getNormal();
  const Cvec3& v = f.getVertex(vertex).getPosition();
  return VertexPN(v[0], v[1], v[2], n[0], n[1], n[2]);
}


// takes a projection matrix and send to the the shaders
inline void sendProjectionMatrix(Uniforms& uniforms, const Matrix4& projMatrix) {
  uniforms.put("uProjMatrix", projMatrix);
}

// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
  if (g_windowWidth >= g_windowHeight)
    g_frustFovY = g_frustMinFov;
  else {
    const double RAD_PER_DEG = 0.5 * CS175_PI/180;
    g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
  }
}

static Matrix4 makeProjectionMatrix() {
  return Matrix4::makeProjection(
           g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
           g_frustNear, g_frustFar);
}

enum ManipMode {
  ARCBALL_ON_PICKED,
  ARCBALL_ON_SKY,
  EGO_MOTION
};

static ManipMode getManipMode() {
  // if nothing is picked or the picked transform is the transfrom we are viewing from
  if (g_currentPickedRbtNode == NULL || g_currentPickedRbtNode == g_currentCameraNode) {
    if (g_currentCameraNode == g_skyNode && g_activeCameraFrame == WORLD_SKY)
      return ARCBALL_ON_SKY;
    else
      return EGO_MOTION;
  }
  else
    return ARCBALL_ON_PICKED;
}

static bool shouldUseArcball() {
  return getManipMode() != EGO_MOTION;
}

// The translation part of the aux frame either comes from the current
// active object, or is the identity matrix when
static RigTForm getArcballRbt() {
  switch (getManipMode()) {
  case ARCBALL_ON_PICKED:
    return getPathAccumRbt(g_world, g_currentPickedRbtNode);
  case ARCBALL_ON_SKY:
    return RigTForm();
  case EGO_MOTION:
    return getPathAccumRbt(g_world, g_currentCameraNode);
  default:
    throw runtime_error("Invalid ManipMode");
  }
}

static void updateArcballScale() {
  RigTForm arcballEye = inv(getPathAccumRbt(g_world, g_currentCameraNode)) * getArcballRbt();
  double depth = arcballEye.getTranslation()[2];
  if (depth > -CS175_EPS)
    g_arcballScale = 0.02;
  else
    g_arcballScale = getScreenToEyeScale(depth, g_frustFovY, g_windowHeight);
}

static void drawArcBall(Uniforms& uniforms) {
  // switch to wire frame mode
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  RigTForm arcballEye = inv(getPathAccumRbt(g_world, g_currentCameraNode)) * getArcballRbt();
  Matrix4 MVM = rigTFormToMatrix(arcballEye) * Matrix4::makeScale(Cvec3(1, 1, 1) * g_arcballScale * g_arcballScreenRadius);
  sendModelViewNormalMatrix(uniforms, MVM, normalMatrix(MVM));

  //safe_glUniform3f(uniforms.h_uColor, 0.27, 0.82, 0.35); // set color
  //g_sphere->draw(uniforms);

  g_arcballMat->draw(*g_sphere, uniforms);
}

///////////////////////// metaball stuff!!!


void drawMetaballs(){
  vector<SurfaceVtx> verts = g_cubeGrid.CreateSurface(g_threshold);
  // covert from surface vertices to geometry
  // dynamic vertex buffer

  vector<VertexPNC> verts2;
  verts2.reserve(3);
  Cvec3 color = Cvec3(0.0, 0.6, 0.5);
 
  for(vector<SurfaceVtx>::iterator it = verts.begin(); it != verts.end(); ++it){

    VertexPNC v = VertexPNC((*it).pos, (*it).normal, color);
    verts2.push_back(v);
  }
  g_cubeGridGeometry->upload(&verts2[0], verts2.size());
  glutPostRedisplay();
}

void updateMetaballs()
{
  //clear the field
  for(int i = 0; i < g_cubeGrid.numVertices; i++)
  {
    g_cubeGrid.vertices[i].value = 0.0;
    g_cubeGrid.vertices[i].normal = Cvec3(0.0);
  }
  
  //evaluate the scalar field at each point
  Cvec3 ballToPoint;
  double radiusSquared;
  Cvec3 ballPosition;
  double normalScale;
  for(int i = 0; i < g_numMetaballs; i++)
  {
    radiusSquared = g_metaballs[i].radiusSquared;
    ballPosition = g_metaballs[i].position;

    for(int j = 0; j < g_cubeGrid.numVertices; j++)
    {
      ballToPoint[0] = g_cubeGrid.vertices[j].pos[0] - ballPosition[0];
      ballToPoint[1] = g_cubeGrid.vertices[j].pos[1] - ballPosition[1];
      ballToPoint[2] = g_cubeGrid.vertices[j].pos[2] - ballPosition[2];
      
      //get squared distance from ball to point
      double distanceSquared = ballToPoint[0]*ballToPoint[0] + ballToPoint[1]*ballToPoint[1] + ballToPoint[2]*ballToPoint[2];
      
      // avoid zero problem
      if(distanceSquared == 0.0)
        distanceSquared = 0.0001;

      //value = r^2/d^2
      g_cubeGrid.vertices[j].value += radiusSquared/distanceSquared;

      //normal = (r^2 * v)/d^4
      normalScale = radiusSquared / (distanceSquared * distanceSquared);

      //cubeGrid.vertices[j].normal+=ballToPoint*normalScale;
      g_cubeGrid.vertices[j].normal[0] += ballToPoint[0] * normalScale;
      g_cubeGrid.vertices[j].normal[1] += ballToPoint[1] * normalScale;
      g_cubeGrid.vertices[j].normal[2] += ballToPoint[2] * normalScale;
    }
  }
  drawMetaballs();
}

void changeMetaballPos(){

  double r = .01*(double)cos(g_timer/2);
  double c = .02*(double)cos((g_timer)/3);
  int cnt = 0;
  for(vector<Metaball>::iterator it = g_metaballs.begin(); it != g_metaballs.end(); ++it){
    double n = double(distance(g_metaballs.begin(), it));
    if(cnt % 10 < 5) {
      (*it).position[0] -= ((.1+r)*(double)sin(2*(g_timer+(n))) - c) / 4; 
      (*it).position[1] -= ((.4+r)*(double)cos((g_timer-4*n)/2) + 2*c) / 4;
      (*it).position[2] -= ((.1+r)*(double)sin(2*(g_timer-n)) - c) / 4;
    }
    else {
      (*it).position[0] += ((.1+r)*(double)sin(2*(g_timer+(2*n))) - c) / 4; 
      (*it).position[1] += ((.4+r)*(double)cos((g_timer-4*n)/2) + 2*c) / 4;
      (*it).position[2] += ((.1+r)*(double)sin(2*(g_timer-n)) - c) / 4;
    }

    cnt++;
  }

  updateMetaballs();
}

// for lava moooovement
static void metaballsTimerCallback(int dontCare) {
    glutTimerFunc(g_animationSpeed, metaballsTimerCallback, 0);

    g_timer += 0.1;
    changeMetaballPos();
}

////////////////////////

static void drawStuff(bool picking) {
  // if we are not translating, update arcball scale
  if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton) || (g_mouseLClickButton && !g_mouseRClickButton && g_spaceDown)))
    updateArcballScale();

  Uniforms uniforms;

  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(uniforms, projmat);

  const RigTForm eyeRbt = getPathAccumRbt(g_world, g_currentCameraNode);
  const RigTForm invEyeRbt = inv(eyeRbt);

  // get world space coordinates of the light
  Cvec3 light1 = getPathAccumRbt(g_world, g_light1).getTranslation();
  Cvec3 light2 = getPathAccumRbt(g_world, g_light2).getTranslation();

  // lamp glass 
  Cvec3 lamp = getPathAccumRbt(g_world, g_lamp).getTranslation();

  // transform to eye space, and set to uLight uniform
  uniforms.put("uLight", Cvec3(invEyeRbt * Cvec4(light1, 1)));
  uniforms.put("uLight2", Cvec3(invEyeRbt * Cvec4(light2, 1)));

  uniforms.put("lamp", Cvec3(invEyeRbt * Cvec4(lamp, 1)));

  if (!picking) {
    Drawer drawer(invEyeRbt, uniforms);
    g_world->accept(drawer);

    if (g_displayArcball && shouldUseArcball())
      drawArcBall(uniforms);
  }
  else {
    Picker picker(invEyeRbt, uniforms);
    g_overridingMaterial = g_pickingMat;
    g_world->accept(picker);
    g_overridingMaterial.reset();
    glFlush();
    g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
    if (g_currentPickedRbtNode == g_groundNode)
      g_currentPickedRbtNode = shared_ptr<SgRbtNode>(); // set to NULL

    cout << (g_currentPickedRbtNode ? "Part picked" : "No part picked") << endl;
  }
}

static void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  drawStuff(false);

  glutSwapBuffers();

  checkGlErrors();
}

static void pick() {
  // We need to set the clear color to black, for pick rendering.
  // so let's save the clear color
  GLdouble clearColor[4];
  glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

  glClearColor(0, 0, 0, 0);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  drawStuff(true);

  // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
  // to see result of the pick rendering pass
  // glutSwapBuffers();

  //Now set back the clear color
  glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

  checkGlErrors();
}

static void reshape(const int w, const int h) {
  g_windowWidth = w;
  g_windowHeight = h;
  glViewport(0, 0, w, h);
  cerr << "Size of window is now " << w << "x" << h << endl;
  g_arcballScreenRadius = max(1.0, min(h, w) * 0.25);
  updateFrustFovY();
  glutPostRedisplay();
}

static Cvec3 getArcballDirection(const Cvec2& p, const double r) {
  double n2 = norm2(p);
  if (n2 >= r*r)
    return normalize(Cvec3(p, 0));
  else
    return normalize(Cvec3(p, sqrt(r*r - n2)));
}

static RigTForm moveArcball(const Cvec2& p0, const Cvec2& p1) {
  const Matrix4 projMatrix = makeProjectionMatrix();
  const RigTForm eyeInverse = inv(getPathAccumRbt(g_world, g_currentCameraNode));
  const Cvec3 arcballCenter = getArcballRbt().getTranslation();
  const Cvec3 arcballCenter_ec = Cvec3(eyeInverse * Cvec4(arcballCenter, 1));

  if (arcballCenter_ec[2] > -CS175_EPS)
    return RigTForm();

  Cvec2 ballScreenCenter = getScreenSpaceCoord(arcballCenter_ec,
                                               projMatrix, g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);
  const Cvec3 v0 = getArcballDirection(p0 - ballScreenCenter, g_arcballScreenRadius);
  const Cvec3 v1 = getArcballDirection(p1 - ballScreenCenter, g_arcballScreenRadius);

  return RigTForm(Quat(0.0, v1[0], v1[1], v1[2]) * Quat(0.0, -v0[0], -v0[1], -v0[2]));
}

static RigTForm doMtoOwrtA(const RigTForm& M, const RigTForm& O, const RigTForm& A) {
  return A * M * inv(A) * O;
}

static RigTForm getMRbt(const double dx, const double dy) {
  RigTForm M;

  if (g_mouseLClickButton && !g_mouseRClickButton && !g_spaceDown) {
    if (shouldUseArcball())
      M = moveArcball(Cvec2(g_mouseClickX, g_mouseClickY), Cvec2(g_mouseClickX + dx, g_mouseClickY + dy));
    else
      M = RigTForm(Quat::makeXRotation(-dy) * Quat::makeYRotation(dx));
  }
  else {
    double movementScale = getManipMode() == EGO_MOTION ? 0.02 : g_arcballScale;
    if (g_mouseRClickButton && !g_mouseLClickButton) {
      M = RigTForm(Cvec3(dx, dy, 0) * movementScale);
    }
    else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton) || (g_mouseLClickButton && g_spaceDown)) {
      M = RigTForm(Cvec3(0, 0, -dy) * movementScale);
    }
  }

  switch (getManipMode()) {
  case ARCBALL_ON_PICKED:
    break;
  case ARCBALL_ON_SKY:
    M = inv(M);
    break;
  case EGO_MOTION:
    if (g_mouseLClickButton && !g_mouseRClickButton && !g_spaceDown) // only invert rotation
      M = inv(M);
    break;
  }
  return M;
}

static RigTForm makeMixedFrame(const RigTForm& objRbt, const RigTForm& eyeRbt) {
  return transFact(objRbt) * linFact(eyeRbt);
}

// l = w X Y Z
// o = l O
// a = w A = l (Z Y X)^1 A = l A'
// o = a (A')^-1 O
//   => a M (A')^-1 O = l A' M (A')^-1 O

static void motion(const int x, const int y) {
  if (!g_mouseClickDown)
    return;

  const double dx = x - g_mouseClickX;
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;

  const RigTForm M = getMRbt(dx, dy);   // the "action" matrix

  // the matrix for the auxiliary frame (the w.r.t.)
  RigTForm A = makeMixedFrame(getArcballRbt(), getPathAccumRbt(g_world, g_currentCameraNode));

  shared_ptr<SgRbtNode> target;
  switch (getManipMode()) {
  case ARCBALL_ON_PICKED:
    target = g_currentPickedRbtNode;
    break;
  case ARCBALL_ON_SKY:
    target = g_skyNode;
    break;
  case EGO_MOTION:
    target = g_currentCameraNode;
    break;
  }

  A = inv(getPathAccumRbt(g_world, target, 1)) * A;

  target->setRbt(doMtoOwrtA(M, target->getRbt(), A));

  g_mouseClickX += dx;
  g_mouseClickY += dy;
  glutPostRedisplay();  // we always redraw if we changed the scene
}

static void mouse(const int button, const int state, const int x, const int y) {
  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

  g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
  g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
  g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

  g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
  g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
  g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

  g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;

  if (g_pickingMode && button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    pick();
    g_pickingMode = false;
    cerr << "Picking mode is off" << endl;
    glutPostRedisplay(); // request redisplay since the arcball will have moved
  }
  glutPostRedisplay();
}

static void keyboardUp(const unsigned char key, const int x, const int y) {
  switch (key) {
  case ' ':
    g_spaceDown = false;
    break;
  }
  glutPostRedisplay();
}

static void keyboard(const unsigned char key, const int x, const int y) {
  switch (key) {
  case ' ':
    g_spaceDown = true;
    break;
  case 27:
    exit(0);                                  // ESC
  case 'h':
    cout << " ============== H E L P ==============\n\n"
    << "h\t\thelp menu\n"
    << "s\t\tsave screenshot\n"
    << "f\t\tToggle flat shading on/off.\n"
    << "p\t\tUse mouse to pick a part to edit\n"
    << "v\t\tCycle view\n"
    << "drag left mouse to rotate\n"
    << "a\t\tToggle display arcball\n"
    << "w\t\tWrite animation to animation.txt\n"
    << "i\t\tRead animation from animation.txt\n"
    << "c\t\tCopy frame to scene\n"
    << "u\t\tCopy sceneto frame\n"
    << "n\t\tCreate new frame after current frame and copy scene to it\n"
    << "d\t\tDelete frame\n"
    << ">\t\tGo to next frame\n"
    << "<\t\tGo to prev. frame\n"
    << "y\t\tPlay/Stop animation\n"
    << endl;
    break;
  case 's':
    glFlush();
    writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
    break;
  case 'a':
    g_displayArcball = !g_displayArcball;
    break;

  glutPostRedisplay();
  }
}

static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
#ifdef __MAC__
  glutInitDisplayMode(GLUT_3_2_CORE_PROFILE|GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH); // core profile flag is required for GL 3.2 on Mac
#else
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
#endif
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Assignment 8");                       // title the window

  glutIgnoreKeyRepeat(true);                              // avoids repeated keyboard calls when holding space to emulate middle mouse

  glutDisplayFunc(display);                               // display rendering callback
  glutReshapeFunc(reshape);                               // window reshape callback
  glutMotionFunc(motion);                                 // mouse movement callback
  glutMouseFunc(mouse);                                   // mouse click callback
  glutKeyboardFunc(keyboard);
  glutKeyboardUpFunc(keyboardUp);
}

static void initGLState() {
  glClearColor(128./255., 200./255., 255./255., 0.);
  glClearDepth(0.);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glReadBuffer(GL_BACK);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initMaterials() {
  // Create some prototype materials
  Material diffuse("./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader");
  Material solid("./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader");

  // copy diffuse prototype and set red color
  g_redDiffuseMat.reset(new Material(diffuse));
  g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));

  // copy diffuse prototype and set blue color
  g_blueDiffuseMat.reset(new Material(diffuse));
  g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0, 1));

  // normal mapping material
  g_bumpFloorMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader"));
  g_bumpFloorMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("spaghetti.ppm", true)));
  g_bumpFloorMat->getUniforms().put("uTexNormal", shared_ptr<ImageTexture>(new ImageTexture("spaghettinormals.ppm", false)));

  // copy solid prototype, and set to wireframed rendering
  g_arcballMat.reset(new Material(solid));
  g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
  g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // copy solid prototype, and set to color white
  g_lightMat.reset(new Material(solid));
  g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));


  // copy solid prototype, and set to color white
  g_lampLightMat.reset(new Material(solid));
  g_lampLightMat->getUniforms().put("uColor", Cvec3f(.5, 0, 1));

  // glass shader
  g_glassMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/glass-gl3.fshader"));

  // metaball texture mapping
  g_balltexMat.reset(new Material("./shaders/metaball-gl3.vshader", "./shaders/metaball-gl3.fshader"));
  g_balltexMat->getUniforms().put("ballColor", Cvec3f(0.5, 0.0, 1.0));

  // pick shader
  g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"));
};

////////////////////////// metaball stuff

static void initCubeGrid() {
  //init cube grid for marching cubes algorithm
  g_cubeGrid.MemoryInit();
  g_cubeGrid.Init(g_gridSize);
  g_cubeGridGeometry.reset(new SimpleGeometryPNC());
}

static void initMetaballs() {
  for(int i= 0; i < g_numMetaballs; i++) {
    Metaball m;
    double radius = g_radiusList[i];
    m.init(Cvec3(double(i)/5.0), radius * radius);
    g_metaballs.push_back(m);
    g_metaballPos.push_back(Cvec3(double(i)/5.0));
  }

}

///////////

static void initGeometry() {
  initGround();
  initCubes();
  initSphere();
  initEllipsoid();
  initMaterials();
  initCubeGrid();
  initMetaballs();
}

static void initScene() {
  g_world.reset(new SgRootNode());

  g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));


  g_groundNode.reset(new SgRbtNode());
  g_groundNode->addChild(shared_ptr<MyShapeNode>(
                           new MyShapeNode(g_ground, g_bumpFloorMat, Cvec3(0, g_groundY, 0))));

  g_light1.reset(new SgRbtNode(RigTForm(Cvec3(0.0, -1.0, 0.0))));
  g_light2.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 10.0, 0.0))));

  g_light1->addChild(shared_ptr<MyShapeNode>(
                           new MyShapeNode(g_sphere, g_lampLightMat, Cvec3(0, g_groundY + 1, -1))));
  g_light2->addChild(shared_ptr<MyShapeNode>(
                           new MyShapeNode(g_sphere, g_lightMat, Cvec3(0, 0, 0))));

  g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
  g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

  g_lamp.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.0, 0.0))));
  g_lamp->addChild(shared_ptr<MyShapeNode>(
                          new MyShapeNode(g_ellipsoid, g_glassMat, Cvec3(0, g_groundY + 5, -1))));

  g_cubeGridNode.reset(new SgRbtNode(RigTForm(Cvec3(0, 0, -2))));
  g_cubeGridNode->addChild(shared_ptr<MyShapeNode>(
                                  new MyShapeNode(g_cubeGridGeometry, g_balltexMat, Cvec3(0), Cvec3(0))));

  g_world->addChild(g_skyNode);
  g_world->addChild(g_groundNode);
  g_world->addChild(g_light1);
  g_world->addChild(g_light2);
  g_world->addChild(g_cubeGridNode);
  g_world->addChild(g_lamp);

  g_currentCameraNode = g_skyNode;
}

static void initAnimation() {
  metaballsTimerCallback(0);
}

int main(int argc, char * argv[]) {
  try {
    initGlutState(argc,argv);

    // on Mac, we shouldn't use GLEW.

#ifndef __MAC__
    glewInit(); // load the OpenGL extensions
#endif

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.5") << endl;

#ifndef __MAC__
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");
#endif

    initGLState();
    initMaterials();
    initGeometry();
    initScene();
    initAnimation();

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
