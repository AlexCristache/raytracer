#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <limits.h>
#include <math.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

//320 x 256

#define SCREEN_WIDTH 960
#define SCREEN_HEIGHT 768
#define FULLSCREEN_MODE false
#define pi 3.1415

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
bool ClosestIntersection(vec4 start, vec4 dir, std::vector<Triangle> triangles, Intersection& target);
mat4 LookAt(vec3 from, vec3 to);
vec3 DirectLight(const Intersection &i);

vec4 camera_pos(0.0, 0.0, -2.5, 1.0);
float yaw = 0.0;
vec4 light_pos( 0.0, -0.5, -0.7, 1.0 );
vec3 light_color = 14.f * vec3( 1, 1, 1 );
std::vector<Triangle> triangles;

int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  while( NoQuitMessageSDL() )
    {
      Update();
      Draw(screen);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  LoadTestModel(triangles);
  vec3 indirectLight = 0.5f * vec3(1,1,1);

  for(float y = 0.0; y < SCREEN_HEIGHT; y++) {
    for(float x = 0.0; x < SCREEN_WIDTH; x++) {
      float focalLength = SCREEN_HEIGHT/2.f;
      float width = SCREEN_WIDTH/2.f;
      float height = SCREEN_HEIGHT/2.f;

      Intersection target;
      vec3 sum = vec3(0);
      for( float dy = -0.5; dy <= 0.5; dy += 0.25 ) {
        for( float dx = -0.5; dx <= 0.5; dx += 0.25 ) {
          vec4 direction = vec4( x + dx - width, y + dy - height, focalLength, 0.f );
          direction = normalize( direction );
          bool found = ClosestIntersection( camera_pos, direction, triangles, target );
          if( found ) {
            vec3 color = DirectLight( target );
            int id = target.triangleIndex;
            sum += triangles[id].color * (color + indirectLight);
          }
        }
      }
      sum = vec3(sum.x / 16, sum.y / 16, sum.z / 16);
      PutPixelSDL( screen, x, y, sum );
    }
  }
}

/*Place updates of parameters here*/
void Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/

  mat4 Rot_y;

  float rad = pi/180.f;
  //vec4 x_col = glm::vec4( cos( yaw * rad ), 0.0, (-1) * sin( yaw * rad) , 0.0 );
  //vec4 y_col = glm::vec4( 0.0, 1.0, 0.0, 0.0 );
  //vec4 z_col = glm::vec4( sin( yaw * rad ), 0.0, cos( yaw * rad ), 0.0 );
  //vec4 translation = glm::vec4( 0.0, 0.0, 0.0, 1.0 );
  //Rot_y = glm::mat4( x_col, y_col, z_col, translation );

  const uint8_t* keystate = SDL_GetKeyboardState( NULL );
  //cout << keystate << endl;
  if( keystate[SDL_SCANCODE_UP] )
  {
    // Move camera forward
    //cout << "up" << endl;
    //camera_pos = camera_pos + vec4( 0, 0, 0.01, 0 );
    //R[2][3] += 0.01;
    //mat4 camToWorld = LookAt( vec3( 1.0, 0.0, -2.5), vec3( 0.0, 0.0, -2.0 ) );
    //camera_pos = camToWorld * camera_pos;
    //translation += glm::vec4( 0.0, 0.0, 0.05, 0.0 );
    //Rot_y = glm::mat4( x_col, y_col, z_col, translation );
    camera_pos = Rot_y * camera_pos;
    //cout << R[2][3] << endl;
    //vec3 from = vec3( 0, 0, -1.5, 1.0);
    //vec3 to = vec3( 0, 0, 0, 0 );
    //mat4 camToWorld = LookAt( from, to );
  }
  if( keystate[SDL_SCANCODE_DOWN] )
  {
    // Move camera backward
    //cout << "down" << endl;
    //translation -= glm::vec4( 0.0, 0.0, 0.05, 0.0 );
    //Rot_y = glm::mat4( x_col, y_col, z_col, translation );
    camera_pos = Rot_y * camera_pos;
  }
  if( keystate[SDL_SCANCODE_LEFT] )
  {
    // Move camera to the left
    cout << "left" << endl;
    yaw = 10.0;
    vec4 x_col = glm::vec4( cos( yaw * rad ), 0.0, sin( yaw * rad * (-1) ) , 0.0 );
    vec4 y_col = glm::vec4( 0.0, 1.0, 0.0, 0.0 );
    vec4 z_col = glm::vec4( sin( yaw * rad ), 0.0, cos( yaw * rad ), 0.0 );
    vec4 translation = glm::vec4( 1.0, 1.0, 1.0, 1.0 );
    Rot_y = glm::mat4( x_col, y_col, z_col, translation );
    /*vec3 from = vec3(camera_pos.x, camera_pos.y, camera_pos.z);
    vec4 to1 = camera_pos * Rot_y;
    vec3 to = vec3( to1.x, to1.y, to1.z );
    mat4 camToWorld = LookAt( from, to);
    for(int j = 0; j < 4; j++) {
        for(int i = 0; i < 4; i++) {
          cout << camToWorld[i][j] << " ";
        }
      cout << endl;
    }
    //vec4 c = vec4(camera.x, camera.y, camera.z, 1.0);
    camera_pos = camera_pos - vec4(camToWorld[2][0], camToWorld[2][1], camToWorld[2][2], 1.0);*/
    camera_pos = camera_pos * Rot_y;
    //cout << "x= " << camera_pos.x << " y= " << camera_pos.y << " z= " << camera_pos.z << " yaw = " << yaw <<  endl;
  }
  if( keystate[SDL_SCANCODE_RIGHT] )
  {
    // Move camera to the right
    cout << "right" << endl;
    yaw = -45.0;
    vec4 x_col = glm::vec4( cos( yaw * rad ), 0.0, sin( yaw * rad ) * (-1), 0.0 );
    vec4 y_col = glm::vec4( 0.0, 1.0, 0.0, 0.0 );
    vec4 z_col = glm::vec4( sin( yaw * rad ), 0.0, cos( yaw * rad ), 0.0 );
    vec4 translation = glm::vec4( 0.0, 0.0, 0.0, 1.0 );
    Rot_y = glm::mat4( x_col, y_col, z_col, translation );
    camera_pos = Rot_y * camera_pos;
  }
}

vec3 DirectLight(const Intersection &i)
{
  vec4 position = i.position;// + vec4(0,0,0,0.08);
  vec4 r = light_pos - position;
  //r.w = 1.0;
  r = normalize(r);
  vec4 n = normalize(triangles[i.triangleIndex].normal);
  float r_dot_n = dot( r, n );
  float radius = length( light_pos - position );
  r_dot_n = max( r_dot_n, 0.f );
  float fraction = r_dot_n / (4.f * pi * radius * radius);
  vec3 D = vec3( light_color.x * fraction, light_color.y * fraction, light_color.z * fraction );

  n = vec4( n.x * 0.001, n.y * 0.001, n.z * 0.001, 0.f );

  Intersection target;
  bool found = ClosestIntersection( position + n, r, triangles, target );
  float d1 = length(position - light_pos);
  float d = length(position - target.position );
  //float a = dot( (position - light_pos), (position - target.position));
  if( found && target.distance < radius ) {
    D = vec3(0.0, 0.0, 0.0);
    //cout << a.x << a.y << a.z << endl;
  }
  return D ;
}

bool ClosestIntersection(vec4 start, vec4 dir, std::vector<Triangle> triangles, Intersection& target)
{
  bool found = false;
  float distance = std::numeric_limits<float>::max();

  for(size_t i = 0; i < triangles.size(); i++) {
    vec4 v0 = triangles[i].v0;
    vec4 v1 = triangles[i].v1;
    vec4 v2 = triangles[i].v2;

    vec3 e1 = vec3((v1.x - v0.x) * 1.0, (v1.y - v0.y) * 1.0, (v1.z - v0.z) * 1.0);
    vec3 e2 = vec3((v2.x - v0.x) * 1.0, (v2.y - v0.y) * 1.0, (v2.z - v0.z) * 1.0);
    vec3 b = vec3((start.x - v0.x) * 1.0, (start.y - v0.y) * 1.0, (start.z - v0.z) * 1.0);
    vec3 direction = vec3(dir.x, dir.y, dir.z);
    direction = normalize(direction);
    mat3 A(-direction, e1, e2);
    vec3 x = glm::inverse(A) * b;

    if( 0 <= x.x && x.x < distance && 0 <= x.y && 0 <= x.z && (x.y + x.z) <= 1 ) {
      distance = x.x;
      found = true;
      vec4 e1_1 = glm::vec4(e1.x, e1.y, e1.z, 0.0);
      vec4 e2_1 = glm::vec4(e2.x, e2.y, e2.z, 0.0);
      target.position = v0 + x.y * e1_1 + x.z * e2_1;
      target.position.w = 1.0;
      target.distance = x.x * 1.0;
      target.triangleIndex = i;
    }
  }

  return found;
}

mat4 LookAt( vec3 from, vec3 to )
{
  vec3 forward = normalize( from - to );

  vec3 randomVec = glm::vec3( 0, -1, 0 );
  vec3 right = cross( normalize(randomVec), forward );

  vec3 up = cross( forward, right );

  mat4 camToWorld;

  vec4 x_col = vec4( right.x, up.x, forward.x, from.x );
  vec4 y_col = vec4( right.y, up.y, forward.y, from.y );
  vec4 z_col = vec4( right.z, up.z, forward.z, from.z );
  //vec4 col = vec4( -dot(right, from), -dot(up, from), -dot(forward, from), 1.0 );
  vec4 col = vec4( 0.0, 0.0, 0.0, 1.0 );

  camToWorld = mat4( x_col, y_col, z_col, col );


  return camToWorld;
}
