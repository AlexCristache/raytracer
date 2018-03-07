#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <limits.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;


#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 384
#define FULLSCREEN_MODE false

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
bool ClosestIntersection(vec4 start, vec4 dir, std::vector<Triangle> triangles, Intersection& target);


vec4 camera_pos(0.0, 0.0, -2.5, 1.0);

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

  /*vec3 colour(1.0,0.0,0.0);
  for(int i=0; i<1000; i++)
    {
      uint32_t x = rand() % screen->width;
      uint32_t y = rand() % screen->height;
      PutPixelSDL(screen, x, y, colour);
    }*/
  std::vector<Triangle> triangles;
  LoadTestModel(triangles);

  for(int y = 0; y < SCREEN_HEIGHT; y++) {
    for(int x = 0; x < SCREEN_WIDTH; x++) {
      float focalLength = SCREEN_HEIGHT/2;
      float width = SCREEN_WIDTH/2;
      float height = SCREEN_HEIGHT/2;
      vec4 direction(x - width, y - height, focalLength, 1.0);

      Intersection target;
      target.triangleIndex = -1;
      bool found = ClosestIntersection(camera_pos, direction, triangles, target);
      if(found) {
        //count++;
        int id = target.triangleIndex;
        PutPixelSDL(screen, x, y, triangles[id].color);
        //std::cout << "x = " << x << " y = " << y << " id = " << id << endl;
      }
    }
  }
  //cout << "-----------------" << endl;
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
  //std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/

  mat4 R;
  const uint8_t* keystate = SDL_GetKeyboardState( NULL );
  cout << keystate << endl;
  if( keystate[SDL_SCANCODE_UP] )
  {
    // Move camera forward
    cout << "up" << endl;
  }
  if( keystate[SDL_SCANCODE_DOWN] )
  {
    // Move camera backward
    cout << "down" << endl;
  }
  if( keystate[SDL_SCANCODE_LEFT] )
  {
    // Move camera to the left
    cout << "left" << endl;
  }
  if( keystate[SDL_SCANCODE_RIGHT] )
  {
    // Move camera to the right
    cout << "right" << endl;
  }
}

bool ClosestIntersection(vec4 start, vec4 dir, std::vector<Triangle> triangles, Intersection& target)
{
  bool found = false;
  float distance = std::numeric_limits<float>::max();

  for(size_t i = 0; i < triangles.size(); i++) {
    vec4 v0 = triangles[i].v0;
    vec4 v1 = triangles[i].v1;
    vec4 v2 = triangles[i].v2;

    vec3 e1 = vec3(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
    vec3 e2 = vec3(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
    vec3 b = vec3(start.x - v0.x, start.y - v0.y, start.z - v0.z);
    vec3 direction(dir.x, dir.y, dir.z);

    mat3 A(-direction, e1, e2);
    vec3 x = glm::inverse(A) * b;

    if(0 <= x.x && x.x < distance && 0 <= x.y && 0 <= x.z && (x.y + x.z) < 1) {
      distance = x.x;
      found = true;
      vec4 e1_1 = glm::vec4(e1.x, e1.y, e1.z, 1.0);
      vec4 e2_1 = glm::vec4(e2.x, e2.y, e2.z, 1.0);
      target.position = v0 + x.y * e1_1 + x.z * e2_1;
      target.distance = x.x;
      target.triangleIndex = (int)i;
    }
  }

  return found;
}
