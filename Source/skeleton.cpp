#include <iostream>
#include <glm/glm.hpp>
//#include "glm/ext.hpp"
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <limits.h>
#include <math.h>
#include "omp.h"
#include "float.h"

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

//320 x 256

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 640
#define FULLSCREEN_MODE false
#define pi 3.1415f

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
bool ClosestIntersection(vec4 start, vec4 dir, std::vector<Triangle> triangles, Intersection& target);
mat4 LookAt(vec3 from, vec3 to);
vec3 DirectLight(const Intersection &i);
void update_rotation_y(glm::mat4& R_y);
glm::vec3 castRay( glm::vec4 origin, glm::vec4 direction, int depth, std::vector<Triangle> triangles, RayType rayType);
//compute the direction of the reflected ray
glm::vec4 ReflectRay(const glm::vec4 IncidentRay, const glm::vec4 Normal);
glm::vec4 refract(const glm::vec4 IncidentRay, const glm::vec4 Normal, float ior);
void fresnel(const glm::vec4& IncidentRay, const glm::vec4& Normal, const float &ior, float &kr );

vec4 camera_pos(0.0, 0.0, -2.1, 1.0);
vec4 camera_pos_y(0.0, 0.0, -2.5, 1.0);
float yaw = 0.0;
mat4 Rot_y;
vec4 light_pos( 0.0, -0.75, -0.0, 1.0 );
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
  cout << triangles.size() << endl;
  RayType rayType = primaryRay;
  int depth;
  bool firstPixel = false;
  #pragma omp parallel for schedule(dynamic, 10)
  for(int y = 0.0; y < SCREEN_HEIGHT; y++) {
    for(int x = 0.0; x < SCREEN_WIDTH; x++) {
      float focalLength = SCREEN_HEIGHT/2.f;
      float width = SCREEN_WIDTH/2.f;
      float height = SCREEN_HEIGHT/2.f;


      glm::vec4 direction;
      //Intersection target;
      glm::vec3 color = glm::vec3(0.f);

      //glm::vec4 direction = vec4( x - width, y - height, focalLength, 1.f );
      // direction = Rot_y * direction;
      // //direction = normalize(direction);
      // bool found = ClosestIntersection( camera_pos, direction, triangles, target, rayType );
      // if( found ) {
      //   vec3 color = DirectLight( target );
      //   int id = target.triangleIndex;
      //   PutPixelSDL( screen, x, y, triangles[id].color * (color + indirectLight));
      // }

      for( float dy = -0.5; dy < 0.5; dy += 0.25 ) {
        for( float dx = -0.5; dx < 0.5; dx += 0.25 ) {
          //vec4 direction = vec4( x + dx - width, y + dy - height, focalLength, 0.f );
          //rotate camera around y axis
          //direction = Rot_y * direction;
          //direction = normalize( direction );
          // direction = vec4( x - 0.5 - width, y - 0.5 - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x - 0.25 - width, y - 0.5 - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x - 0.0f - width, y - 0.5 - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x + 0.25 - width, y - 0.5 - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x + 0.5 - width, y - 0.5 - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x - 0.5 - width, y - 0.25 - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x - 0.25 - width, y - 0.25 - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x - 0.0f - width, y - 0.25 - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x + 0.25 - width, y - 0.25 - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x + 0.5 - width, y - 0.25 - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x - 0.5 - width, y - 0.0f - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x - 0.25 - width, y - 0.0f - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x - 0.0f - width, y - 0.0f - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x + 0.25 - width, y - 0.0f - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x + 0.25 - width, y + 0.25 - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);
          //
          // direction = vec4( x - 0.25 - width, y + 0.25 - height, focalLength, 1.f );
          // depth = 0;
          // color += castRay( camera_pos, direction, depth, triangles, primaryRay);

          direction = vec4( (float)x + dx - width, (float)y + dy - height, focalLength, 1.0f );
          depth = 0;
          color += castRay( camera_pos, direction, depth, triangles, primaryRay);

          // bool found = ClosestIntersection( camera_pos, direction, triangles, target, rayType );
          //
          // if( found ) {
          //   vec3 color = DirectLight( target );
          //   int id = target.triangleIndex;
          //   sum += triangles[id].color * (color + indirectLight);
          // }
        }
      }
      color = vec3(color.x/16, color.y/16, color.z/16);
      PutPixelSDL( screen, x, y, color );
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

  const uint8_t* keystate = SDL_GetKeyboardState( NULL );
  //cout << keystate << endl;
  if( keystate[SDL_SCANCODE_W] )
  {
    camera_pos.z += 0.2;
  }
  if( keystate[SDL_SCANCODE_S] )
  {
    camera_pos.z -= 0.2;
  }
  if( keystate[SDL_SCANCODE_A] )
  {
    camera_pos.x -= 0.2;
  }
  if( keystate[SDL_SCANCODE_D] )
  {
    camera_pos.x += 0.2;
  }
  if( keystate[SDL_SCANCODE_LEFT] )
  {
    // Move camera to the left
    cout << "left" << endl;
    yaw += 0.05;
    update_rotation_y(Rot_y);
    //camera_pos = Rot_y * camera_pos;
    //camera_pos_y = Rot_y * camera_pos;
  }
  if( keystate[SDL_SCANCODE_RIGHT] )
  {
    // Move camera to the right
    cout << "right" << endl;
    yaw -= 0.05;
    update_rotation_y(Rot_y);
    //camera_pos = Rot_y * camera_pos;
    //camera_pos_y = Rot_y * camera_pos;
  }
  if( keystate[SDL_SCANCODE_I] )
  {
    light_pos.z += 0.2;
  }
  if( keystate[SDL_SCANCODE_K] )
  {
    light_pos.z -= 0.2;
  }
  if( keystate[SDL_SCANCODE_J] )
  {
    light_pos.x -= 0.2;
  }
  if( keystate[SDL_SCANCODE_L] )
  {
    light_pos.x += 0.2;
  }
  if( keystate[SDL_SCANCODE_U] )
  {
    light_pos.y -= 0.2;
  }
  if( keystate[SDL_SCANCODE_O] )
  {
    light_pos.y += 0.2;
  }
}

//Rotate the camera view around the Y axis.
void update_rotation_y (glm::mat4& R_y)
{
  R_y =  glm::mat4 (cos(yaw), 0, sin(yaw), 0,
                       0,     1,     0,    0,
                  -sin(yaw), 0, cos(yaw), 0,
                       0,     0,     0,    1);
}

vec3 DirectLight(const Intersection &i)
{
  vec4 position = i.position;// + vec4(0,0,0,0.08);
  vec4 r = light_pos - position;
  //r.w = 1.0;
  r = glm::normalize(r);
  vec4 n = glm::normalize(triangles[i.triangleIndex].normal);
  float r_dot_n = glm::dot( r, n );
  float radius = length( light_pos - position );
  r_dot_n = max( r_dot_n, 0.f );
  float fraction = r_dot_n / (4.f * pi * radius * radius);
  vec3 D = vec3( light_color.x * fraction, light_color.y * fraction, light_color.z * fraction );

  n = vec4( n.x * 0.001, n.y * 0.001, n.z * 0.001, 0.f );

  Intersection target;
  RayType rayType = shadowRay;
  bool found = ClosestIntersection( position + n, r, triangles, target);
  //float d1 = length(position - light_pos);
  //float d = length(position - target.position );
  //float a = dot( (position - light_pos), (position - target.position));
  if( found && target.distance < radius ) {
    D = vec3(0.0, 0.0, 0.0);
    //cout << a.x << a.y << a.z << endl;
  }
  return D ;
}

//compute the direction of the reflected ray
glm::vec4 ReflectRay(const glm::vec4 IncidentRay, const glm::vec4 Normal)
{
  glm::vec3 i = glm::vec3( IncidentRay.x, IncidentRay.y, IncidentRay.z );
  glm::vec3 n = glm::vec3( Normal.x, Normal.y, Normal.z );
  glm::vec3 result = i - 2 * glm::dot(i, n) * n;
  result = glm::normalize(result);
  return glm::vec4(result, 1.0f);
  //result.w = 1.f;
  //return result;
}

glm::vec4 refract(const glm::vec4 IncidentRay, const glm::vec4 Normal, float ior)
{
  glm::vec3 i = glm::vec3( IncidentRay.x, IncidentRay.y, IncidentRay.z );
  glm::vec3 n = glm::vec3(Normal.x, Normal.y, Normal.z);
  float cosi = glm::clamp(-1.0f, 1.0f, glm::dot(i, n));
  float etai = 1;
  float etat = ior;

  if( cosi < 0 ) {
    cosi = -cosi;
  }
  else {
    std::swap( etai, etat );
    n = -n;
  }

  float eta = etai/etat;
  float k = 1 - eta * eta * ( 1 - cosi * cosi );

  return k < 0 ? glm::vec4(0) : glm::vec4(glm::normalize(eta * i + (eta * cosi - sqrtf(k)) * n), 1.0);
}

void fresnel(const glm::vec4& IncidentRay, const glm::vec4& Normal, const float &ior, float &kr )
{
  glm::vec3 i = glm::vec3( IncidentRay.x, IncidentRay.y, IncidentRay.z );
  glm::vec3 n = glm::vec3( Normal.x, Normal.y, Normal.z );
  float cosi = glm::clamp( -1.0f, 1.0f, glm::dot( i, n ) );
  float etai = 1.0f;

  float etat = ior;

  if( cosi > 0 ) {
    std::swap( etai, etat );
  }

  float sint = etai / etat * sqrtf( std::max( 0.f, 1 - cosi * cosi ) );
  if( sint >= 1 ) {
    kr = 1;
  }
  else {
    float cost = sqrtf( std::max( 0.f, 1 - sint * sint ) );
    cosi = fabsf( cosi );
    float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
    float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
    kr = (Rs * Rs + Rp * Rp) / 2.0f;
  }
}

bool ClosestIntersection(vec4 start, vec4 dir, std::vector<Triangle> triangles, Intersection& target)
{
  bool found = false;
  float distance = std::numeric_limits<float>::max();
  vec3 direction = vec3(dir.x, dir.y, dir.z);
  direction = normalize(direction);

  for(size_t i = 0; i < triangles.size(); i++) {
    vec4 v0 = triangles[i].v0;
    vec4 v1 = triangles[i].v1;
    vec4 v2 = triangles[i].v2;

    vec3 e1 = vec3((v1.x - v0.x) * 1.0f, (v1.y - v0.y) * 1.0f, (v1.z - v0.z) * 1.0f);
    vec3 e2 = vec3((v2.x - v0.x) * 1.0f, (v2.y - v0.y) * 1.0f, (v2.z - v0.z) * 1.0f);
    vec3 b = vec3((start.x - v0.x) * 1.0f, (start.y - v0.y) * 1.0f, (start.z - v0.z) * 1.0f);
    mat3 A(-direction, e1, e2);

    // float det_A = glm::determinant(A);
    // mat3 A_i = A;
    // A_i[0] = b;
    // float t = glm::determinant(A_i)/det_A;
    // if(t < FLT_EPSILON * (1e-4)) {
    //   continue;
    // }
    //
    // A_i[0] = A[0];
    // A_i[1] = b;
    // float u = glm::determinant(A_i)/det_A;
    // if(u < FLT_EPSILON * (1e-4)) {
    //   continue;
    // }
    //
    // A_i[1] = A[1];
    // A_i[2] = b;
    // float v = glm::determinant(A_i)/det_A;
    // if(v < FLT_EPSILON * (1e-4)) {
    //   continue;
    // }
    //
    // if(v < 0.0f || (u + v) > 1.0f) {
    //   continue;
    // }
    // vec4 position = v0 + (glm::vec4(e1, 0.0f) * u) + (glm::vec4(e2, 0.0f) * v);
    // position.w = 1.0f;
    // float r = length(position - start);
    vec3 x = glm::inverse(A) * b;
    if( 0 <= x.x && x.x < distance && 0 <= x.y && 0 <= x.z && (x.y + x.z) <= 1 ) {
    //if(t < distance) {
      distance = x.x;
      found = true;
      vec4 e1_1 = glm::vec4(e1.x, e1.y, e1.z, 0.0);
      vec4 e2_1 = glm::vec4(e2.x, e2.y, e2.z, 0.0);
      target.position = v0 + x.y * e1_1 + x.z * e2_1;
      target.position.w = 1.0;
      target.distance = x.x * 1.0;
      target.triangleIndex = i;
      // distance = t;
      // target.position = position;
      // target.distance = r;
      // target.triangleIndex = i;
      // found = true;
    }
  }

  //if a closest intersection was found
  // if(found && rayType == primaryRay) {
  //   //check for surface properties
  //   switch (triangles[target.triangleIndex].material) {
  //     case kReflection: {
  //       //cout << "triangle #" << target.triangleIndex << endl;
  //       glm::vec4 incidentRay = glm::vec4(direction, 1.0);
  //       glm::vec4 reflectedRay = ReflectRay(incidentRay, triangles[target.triangleIndex].normal);
  //       Intersection reflectedTarget;
  //       //cout << "#1 " << endl;
  //       glm::vec4 origin = target.position + glm::vec4(0.001) * triangles[target.triangleIndex].normal;
  //       origin.w = 1.0;
  //       bool reflectionFound = ClosestIntersection(origin, reflectedRay, triangles, reflectedTarget, rayType);
  //       //cout << "#2 " << reflectionFound << endl;
  //       if(reflectionFound) {
  //         target.triangleIndex = reflectedTarget.triangleIndex;
  //       }
  //       return reflectionFound;
  //       break;
  //     }
  //     case kReflectionAndRefraction: {
  //       float kr;
  //       glm::vec4 incidentRay = glm::vec4(direction, 1.0);
  //       glm::vec4 origin = target.position + glm::vec4(0.001) * triangles[target.triangleIndex].normal;
  //       origin.w = 1.0;
  //       return found;
  //       break;
  //     }
  //     default: {
  //       return found;
  //       break;
  //     }
  //   }
  // }
  return found;
}

glm::vec3 castRay( glm::vec4 origin, glm::vec4 direction, int depth, std::vector<Triangle> triangles, RayType rayType)
{
  Intersection target;
  glm::vec3 indirectLight = 0.5f * glm::vec3(1,1,1);
  glm::vec3 color = glm::vec3(0.f);
  if( depth > 4 ) {
    return glm::vec3( 0, 0, 0 );
  }
  bool found = ClosestIntersection( origin, direction, triangles, target);
  if( found ) {
    switch (triangles[target.triangleIndex].material) {
      case kDiffuse: {
        // glm::vec3 iluminated_color = DirectLight( target );
        // color += triangles[target.triangleIndex].color * (iluminated_color + indirectLight);
        vec3 diffuse = glm::vec3(0.0f);
        vec3 specular = glm::vec3(0.0f);
        glm::vec3 light_direction = glm::vec3((light_pos - target.position).x, (light_pos - target.position).y, (light_pos - target.position).z);
        glm::vec3 normal = glm::vec3(triangles[target.triangleIndex].normal.x, triangles[target.triangleIndex].normal.y, triangles[target.triangleIndex].normal.z);

        glm::vec4 reflectedRay = ReflectRay(light_pos-target.position, triangles[target.triangleIndex].normal);
        glm::vec3 r = glm::vec3(reflectedRay.x, reflectedRay.y, reflectedRay.z);
        vec3 light_intensity = DirectLight(target);

        light_direction = normalize(light_direction);
        float dot_prod = glm::dot(normal, light_direction);
        diffuse = glm::vec3(0.68f) * light_intensity * std::max(0.f, dot_prod);


        vec3 dir = glm::vec3(direction.x, direction.y, direction.z);
        dir = normalize(dir);
        float r_dot_dir = glm::dot(r, dir);
        specular = light_intensity * std::pow(std::max(0.f, r_dot_dir), 10.0f);
        color += triangles[target.triangleIndex].color + diffuse * 0.6f + specular * 0.4f;
        break;
      }
      case kReflection: {
        glm::vec3 iluminated_color = DirectLight( target );
        color += triangles[target.triangleIndex].color * (iluminated_color + indirectLight);
        //cout << "triangle #" << target.triangleIndex << endl;
        // vec3 rayDirection = vec3(direction.x, direction.y, direction.z);
        // rayDirection = normalize(rayDirection);
        // glm::vec4 incidentRay = glm::vec4(rayDirection, 1.0);
        // glm::vec4 reflectedRay = ReflectRay(incidentRay, triangles[target.triangleIndex].normal);
        // Intersection reflectedTarget;
        // //cout << "#1 " << endl;
        // glm::vec4 origin = target.position + glm::vec4(0.001) * triangles[target.triangleIndex].normal;
        // origin.w = 1.0;
        // color += castRay(origin, reflectedRay, depth + 1, triangles, rayType);
        // //cout << "#2 " << reflectionFound << endl;
        // glm::vec3 iluminated_color = DirectLight( target );
        // color = color * (iluminated_color + indirectLight);
        //return color * (iluminated_color + indirectLight);
        break;
      }
      case kReflectionAndRefraction: {
        glm::vec3 refractionColor = glm::vec3(0.f);
        glm::vec3 reflectionColor = glm::vec3(0.f);
        float kr;
        Triangle hitTriangle = triangles[target.triangleIndex];
        fresnel( direction, hitTriangle.normal, hitTriangle.ior, kr);
        bool outside = glm::dot(direction, hitTriangle.normal) < 0;
        glm::vec4 bias = 0.001f * hitTriangle.normal;
        //cout << "kr = " << kr << endl;
        if( kr < 1 ) {
          glm::vec4 refractionDirection = refract(direction, hitTriangle.normal, hitTriangle.ior);
          glm::vec4 refractionRayOrigin = outside ? target.position - bias : target.position + bias;

          refractionColor = castRay( refractionRayOrigin, refractionDirection, depth + 1, triangles, rayType );
        }

        glm::vec4 reflectionDirection = reflect(direction, hitTriangle.normal);
        glm::vec4 reflectionRayOrigin = outside ? target.position + bias : target.position - bias;
        reflectionColor = castRay( reflectionRayOrigin, reflectionDirection, depth + 1, triangles, rayType);

        color += reflectionColor * kr + refractionColor * (1 - kr);
        break;
      }
      default: {

        glm::vec3 iluminated_color = DirectLight( target );
        color += triangles[target.triangleIndex].color * (iluminated_color + indirectLight);
        //return triangles[target.triangleIndex].color * (iluminated_color + indirectLight);
        break;
      }
    }
  }
  else {
    color = glm::vec3(0.f);
  }

  return color;// /= depth * 1.f + 1.f;
}
// mat4 LookAt( vec3 from, vec3 to )
// {
//   vec3 forward = normalize( from - to );
//
//   vec3 randomVec = glm::vec3( 0, -1, 0 );
//   vec3 right = cross( normalize(randomVec), forward );
//
//   vec3 up = cross( forward, right );
//
//   mat4 camToWorld;
//
//   vec4 x_col = vec4( right.x, up.x, forward.x, from.x );
//   vec4 y_col = vec4( right.y, up.y, forward.y, from.y );
//   vec4 z_col = vec4( right.z, up.z, forward.z, from.z );
//   //vec4 col = vec4( -dot(right, from), -dot(up, from), -dot(forward, from), 1.0 );
//   vec4 col = vec4( 0.0, 0.0, 0.0, 1.0 );
//
//   camToWorld = mat4( x_col, y_col, z_col, col );
//
//
//   return camToWorld;
// }
