#include <iostream>
#include <glm/glm.hpp>
#include "glm/gtc/random.hpp"
#include "glm/ext.hpp"
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

#define SCREEN_WIDTH 300
#define SCREEN_HEIGHT 300
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
std::vector<Photon> generatePhotonMap(int number_of_photons);
glm::vec3 collect_photons(glm::vec4 position, Triangle& triangle);
void generateLight();

void print_progress_bar(float progress);

vec4 camera_pos(0.6, 0.0, -3.0, 1.0);
vec4 camera_pos_y(0.8, 0.0, -2.5, 1.0);
float yaw = 0.15;
mat4 Rot_y;
vec4 light_pos( 0.0, -0.5, -0.7, 1.0 );
vec3 light_color = 14.f * vec3( 1, 1, 1 );
glm::vec3 indirectLight = 0.5f * glm::vec3(1,1,1);
std::vector<Triangle> triangles;
std::vector<Photon> photons;
std::vector<vec4> light_samples;

int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  LoadTestModel(triangles);
  generateLight();
  photons = generatePhotonMap(100000);

  //cout << "number of photons " << photons.size() << endl;

  omp_set_num_threads(6);

  while( NoQuitMessageSDL() )
    {
      Update();
      Draw(screen);
      SDL_Renderframe(screen);
    }

    SDL_SaveImage( screen, "gi+aa+ss+mirror.bmp" );

    KillSDL(screen);
  cout << "DONE" << endl;
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  RayType rayType = primaryRay;
  int depth;
  int pixels_processed = 0;
  int total_pixels = SCREEN_HEIGHT * SCREEN_WIDTH;

  #pragma omp parallel for
  for(int y = 0; y < SCREEN_HEIGHT; y++) {
    for(int x = 0; x < SCREEN_WIDTH; x++) {
      float focalLength = SCREEN_HEIGHT;
      float width = SCREEN_WIDTH/2.f;
      float height = SCREEN_HEIGHT/2.f;

      glm::vec3 color = glm::vec3(0.f);
      depth = 0;
      for( float dy = -0.5; dy < 0.5; dy += 0.25 ) {
        for( float dx = -0.5; dx < 0.5; dx += 0.25 ) {
          glm::vec4 direction = vec4( (float)x + dx - width, (float)y + dy - height, focalLength, 1.0f );
          depth = 0;
          update_rotation_y(Rot_y);
          direction = Rot_y * direction;
          color += castRay( camera_pos, direction, depth, triangles, rayType);
        }
      }
      color /= 16.0f;
      PutPixelSDL( screen, x, y, color );
      pixels_processed++;
    }
    cout << (float)pixels_processed/total_pixels * 100.f << "%\r";
    cout.flush();
  }
  cout << endl;
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
  }
  if( keystate[SDL_SCANCODE_RIGHT] )
  {
    // Move camera to the right
    cout << "right" << endl;
    yaw -= 0.05;
    update_rotation_y(Rot_y);
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

void generateLight()
{
  light_samples.push_back(light_pos);
  for(int i = 1; i < 20; i++) {
    glm::vec4 light_sample = light_pos;
    float dx = glm::linearRand<float>(-0.02f, 0.02f);
    float dz = glm::linearRand<float>(-0.02f, 0.02f);
    float dy = glm::linearRand<float>(-0.02f, 0.02f);
    light_sample.x += dx;
    light_sample.z += dz;
    light_sample.y += dy;

    light_samples.push_back(light_sample);
  }
}

vec3 DirectLight(const Intersection &i)
{
  vec4 position = i.position;
  vec3 D = vec3(0.0f);
  for(int j = 0; j < 20; j++) {
    vec3 color = vec3(0.0f);

    vec4 r = light_samples[j] - position;
    r = glm::normalize(r);
    vec4 n = glm::normalize(triangles[i.triangleIndex].normal);
    float r_dot_n = glm::dot( r, n );
    float radius = length( light_samples[j] - position );
    r_dot_n = max( r_dot_n, 0.f );
    float fraction = r_dot_n / (4.f * pi * radius * radius);
    color += vec3( light_color.x * fraction, light_color.y * fraction, light_color.z * fraction );



    n = vec4( n.x * 0.001, n.y * 0.001, n.z * 0.001, 0.f );

    Intersection target;
    bool found = ClosestIntersection( position + n, r, triangles, target);
    if( found && target.distance <= radius ) {
      color = vec3(0.0, 0.0, 0.0);
    }

    D += color;
  }
  D = vec3(D.x/20.0f, D.y/20.0f, D.z/20.0f);
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

    /* Cramer's Rule */
    // float det_A = glm::determinant(A);
    // mat3 A_i = A;
    // A_i[0] = b;
    // float t = glm::determinant(A_i)/det_A;
    // if(t < 0) {
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
    // if(v < 0) {
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
      }
  }

  return found;
}

glm::vec3 castRay( glm::vec4 origin, glm::vec4 direction, int depth, std::vector<Triangle> triangles, RayType rayType)
{
  Intersection target;

  glm::vec3 color = glm::vec3(0.f);
  if( depth > 4 ) {
    return glm::vec3( 0, 0, 0 );
  }
  bool found = ClosestIntersection( origin, direction, triangles, target);
  if( found ) {
    switch (triangles[target.triangleIndex].material) {
      case kDiffuse: {
        vec3 diffuse = glm::vec3(0.0f);
        vec3 specular = glm::vec3(0.0f);
        glm::vec3 light_direction = glm::vec3((light_pos - target.position).x, (light_pos - target.position).y, (light_pos - target.position).z);
        glm::vec3 normal = glm::vec3(triangles[target.triangleIndex].normal.x, triangles[target.triangleIndex].normal.y, triangles[target.triangleIndex].normal.z);

        glm::vec4 reflectedRay = ReflectRay(light_pos-target.position, triangles[target.triangleIndex].normal);
        glm::vec3 r = glm::vec3(reflectedRay.x, reflectedRay.y, reflectedRay.z);
        vec3 light_intensity = DirectLight(target);

        light_direction = normalize(light_direction);
        float dot_prod = glm::dot(normal, light_direction);
        diffuse = glm::vec3(0.18f) * light_intensity * std::max(0.f, dot_prod);


        vec3 dir = glm::vec3(direction.x, direction.y, direction.z);
        dir = normalize(dir);
        float r_dot_dir = glm::dot(r, dir);
        specular = light_intensity * std::pow(std::max(0.f, r_dot_dir), 10.0f);
        color += triangles[target.triangleIndex].color + diffuse * 0.6f + specular * 0.4f;
        break;
      }
      case kReflection: {
        vec3 rayDirection = vec3(direction.x, direction.y, direction.z);
        rayDirection = normalize(rayDirection);
        glm::vec4 incidentRay = glm::vec4(rayDirection, 1.0);
        glm::vec4 reflectedRay = ReflectRay(incidentRay, triangles[target.triangleIndex].normal);
        Intersection reflectedTarget;
        glm::vec4 origin = target.position + glm::vec4(0.001) * triangles[target.triangleIndex].normal;
        origin.w = 1.0;
        color += castRay(origin, reflectedRay, depth + 1, triangles, rayType);
        glm::vec3 iluminated_color = DirectLight( target );
        color = color * (iluminated_color + indirectLight);
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
        glm::vec3 photon_color = collect_photons(target.position, triangles[target.triangleIndex]);

        color += triangles[target.triangleIndex].color * (iluminated_color + indirectLight) + photon_color;
        break;
      }
    }
  }
  else {
    color = glm::vec3(0.f);
  }

  return color;
}

std::vector<Photon> generatePhotonMap(int number_of_photons)
{
  std::vector<Photon> photonMap;

  for(int i = 0; i < number_of_photons; i++) {//go through all the photons
    glm::vec4 position = light_pos;
    glm::vec4 direction = glm::linearRand<float>(glm::vec4(-1.0f), glm::vec4(1.0f));
    direction.w = 1.0f;
    glm::vec3 color = glm::vec3(1.0f);
    RayType photon_rayType = primaryRay;
    Intersection photon_target;
    for(int bounces = 0; bounces < 5; bounces++) {//allow 5 bounces per photon
      //check if the photon hits any geometry
      bool photon_target_found = ClosestIntersection(position, direction, triangles, photon_target);
      if(photon_target_found) { //if it does

        position = photon_target.position;
        Photon photon;
        photon.position = position;

        photon.normal = triangles[photon_target.triangleIndex].normal;
        direction = triangles[photon_target.triangleIndex].normal;;

        direction = glm::rotateX(direction, glm::linearRand(-0.99f, 0.99f) * pi / 2.0f);
        direction = glm::rotateY(direction, glm::linearRand(-0.99f, 0.99f) * pi / 2.0f);
        direction = glm::rotateZ(direction, glm::linearRand(-0.99f, 0.99f) * pi / 2.0f);
        glm::vec3 dir = glm::vec3(direction.x, direction.y, direction.z);
        dir = normalize(dir);
        direction = glm::vec4(dir, 1.0f);

        color.x *= triangles[photon_target.triangleIndex].color.x;
        color.y *= triangles[photon_target.triangleIndex].color.y;
        color.z *= triangles[photon_target.triangleIndex].color.z;
        photon.color = glm::clamp(color, glm::vec3(0.0f), glm::vec3(1.0f));

        if(photon_rayType != primaryRay) {//and it's not a primary ray aka direct light
          photonMap.push_back(photon);
        }
        position.x += dir.x * 0.001f;
        position.y += dir.y * 0.001f;
        position.z += dir.z * 0.001f;

        photon_rayType = secondaryRay;
      }
      else break;
    }
  }
  return photonMap;
}

glm::vec3 collect_photons(glm::vec4 position, Triangle& triangle)
{
  glm::vec3 color = glm::vec3(0.0f);
  int count = 0;
  #pragma omp parallel for
  for(int i = 0; i < photons.size(); i++) {
    if(glm::distance(photons[i].position, position) < 0.4f) {
      glm::vec3 p_n = glm::vec3(photons[i].normal.x, photons[i].normal.y, photons[i].normal.z);
      glm::vec3 t_n = glm::vec3(triangle.normal.x, triangle.normal.y, triangle.normal.z);

      float angle = glm::dot(p_n, t_n);
      if(angle > 0.25 * length(t_n) * length(p_n)) {
        color += photons[i].color;
        count++;
      }
    }
  }

  color /= (2.4f * count);
  return glm::clamp(color, glm::vec3(0.f), glm::vec3(1.f)) * 1.5f;
}
