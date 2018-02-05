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


#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
bool ClosestIntersection(glm::vec4 start, glm::vec3 dir, std::vector<Triangle> triangles, Intersection target);

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

  std::vector<Triangle> triangles;

  LoadTestModel(triangles);
  /*vec3 colour(1.0,0.0,0.0);
  for(int i=0; i<1000; i++)
    {
      uint32_t x = rand() % screen->width;
      uint32_t y = rand() % screen->height;
      PutPixelSDL(screen, x, y, colour);
    }
  */
  float focalLength = SCREEN_WIDTH;
  glm::vec4 camera_pos = glm::vec4(0, 0, -3, 1.0);
  for(int y = 0; y < SCREEN_HEIGHT; y++) {
    for(int x = 0; x < SCREEN_WIDTH; x++) {
      glm::vec3 d = glm::vec3(x - SCREEN_WIDTH/2, y - SCREEN_HEIGHT/2, focalLength);
      //glm::vec3 d = glm::vec3(x - 0, y - 0, -3);

      Intersection target;
      target.distance = -100;
      target.position = glm::vec4(-100, -100, -100, -100);
      target.triangleIndex = -1000;

      bool found = ClosestIntersection(camera_pos, d, triangles, target);
      if( found ) {
        glm::vec4 position = target.position;
        uint32_t u = focalLength * (position.x / target.distance) + SCREEN_WIDTH/2;
        uint32_t v = focalLength * (position.y / target.distance) + SCREEN_HEIGHT/2;
        cout << " U@@@@ " << u << endl;
        cout << " V@@@@ " << v << endl;

        glm::vec3 color = triangles[target.triangleIndex].color;
        PutPixelSDL(screen, u, v, color);
      }
    }
  }
}

/*Place updates of parameters here*/
void Update()
{
  //static int t = SDL_GetTicks();
  /* Compute frame time */
  //int t2 = SDL_GetTicks();
  //float dt = float(t2-t);
  //t = t2;
  /*Good idea to remove this*/
  //std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
}

bool ClosestIntersection(glm::vec4 start, glm::vec3 dir, std::vector<Triangle> triangles, Intersection target)
{
  float m = std::numeric_limits<float>::max();
  glm::vec4 pos;
  pos.w = 1.0;
  bool found = false;
  int triangleInd = target.triangleIndex = 0;

  for(int i = 0; i < (int) triangles.size(); i++) {
    glm::vec4 v0 = triangles[i].v0;
    glm::vec4 v1 = triangles[i].v1;
    glm::vec4 v2 = triangles[i].v2;

    glm::vec3 e1 = glm::vec3(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
    glm::vec3 e2 = glm::vec3(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
    glm::vec3 b = glm::vec3(start.x - v0.x, start.y - v0.y, start.z - v0.z);

    glm::mat3 A(-vec3(dir), e1, e2);
    glm::vec3 x = glm::inverse( A ) * b;


    if(x.x < m && x.y >= 0 && x.z >= 0 && (x.y + x.z) <= 1) {
      m = x.x;
      found = true;
      glm::vec4 e1_1 = glm::vec4(e1.x, e1.y, e1.z, 0);
      glm::vec4 e2_1 = glm::vec4(e2.x, e2.y, e2.z, 0);
      glm::vec4 r = v0 + x.y * e1_1 + x.z * e2_1;
      pos = r;
      triangleInd = i;
      cout << " found " << endl;
    }
  }

  if(found) {
    target.position = pos;
    target.distance = m;
    target.triangleIndex = triangleInd;
    cout << " int position " << target.position.x << " " << target.position.y << endl;
  }


  return found;
}
