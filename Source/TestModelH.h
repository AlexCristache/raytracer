#ifndef TEST_MODEL_CORNEL_BOX_H
#define TEST_MODEL_CORNEL_BOX_H

// Defines a simple test model: The Cornel Box

#include <glm/glm.hpp>
#include <vector>
#include <fstream>
enum MaterialType { kDiffuse, kReflection, kReflectionAndRefraction, kDefault };
enum RayType { primaryRay, shadowRay, secondaryRay };

// Used to describe a triangular surface:
class Triangle
{
public:
	glm::vec4 v0;
	glm::vec4 v1;
	glm::vec4 v2;
	glm::vec4 normal;
	glm::vec3 color;
	MaterialType material;
	float ior;

	Triangle( glm::vec4 v0, glm::vec4 v1, glm::vec4 v2, glm::vec3 color )
		: v0(v0), v1(v1), v2(v2), color(color)
	{
		material = kDefault;
		ior = 1.8f;
		ComputeNormal();
	}

	Triangle( glm::vec4 v0, glm::vec4 v1, glm::vec4 v2, glm::vec3 color, MaterialType material )
		: v0(v0), v1(v1), v2(v2), color(color), material(material)
	{
		ior = 1.8f;
		ComputeNormal();
	}

	void ComputeNormal()
	{
	  glm::vec3 e1 = glm::vec3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
	  glm::vec3 e2 = glm::vec3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
	  glm::vec3 normal3 = glm::normalize( glm::cross( e2, e1 ) );
	  normal.x = normal3.x;
	  normal.y = normal3.y;
	  normal.z = normal3.z;
	  normal.w = 1.0;
	}
};

//define the Intersection between a ray and a triangle
struct Intersection
{
	glm::vec4 position;
	float distance;
	int triangleIndex;
};

//define the structure of a photon
struct Photon
{
	glm::vec4 position;
	glm::vec3 color;
	glm::vec4 normal;
};

// Loads the Cornell Box. It is scaled to fill the volume:
// -1 <= x <= +1
// -1 <= y <= +1
// -1 <= z <= +1
void LoadTestModel( std::vector<Triangle>& triangles )
{
	using glm::vec3;
	using glm::vec4;

	// Defines colors:
	vec3 red(    0.75f, 0.15f, 0.15f );
	vec3 yellow( 0.75f, 0.75f, 0.15f );
	vec3 green(  0.15f, 0.75f, 0.15f );
	vec3 cyan(   0.15f, 0.75f, 0.75f );
	vec3 blue(   0.15f, 0.15f, 0.75f );
	vec3 purple( 0.75f, 0.15f, 0.75f );
	vec3 white(  0.75f, 0.75f, 0.75f );
	vec3 gold(   0.95f, 0.85f, 0.0f );
	vec3 dark_green( 0.0f, 0.45f, 0.1f);
	vec3 grey( 0.41f, 0.41f, 0.41f);
	vec3 dark_turquoise( 0.0f, 0.8f, 0.81f);
	vec3 metallic_gold( 0.83f, 0.69f, 0.22f);
	vec3 midnight_blue( 0.01f, 0.01f, 0.44f);
	vec3 ivory( 0.4f, 0.4f, 0.4f);

	triangles.clear();
	triangles.reserve( 5*2*3 );

	// ---------------------------------------------------------------------------
	// Room

	float L = 555;			// Length of Cornell Box side.

	vec4 A(L,0,0,1);
	vec4 B(0,0,0,1);
	vec4 C(L,0,L,1);
	vec4 D(0,0,L,1);

	vec4 E(L,L,0,1);
	vec4 F(0,L,0,1);
	vec4 G(L,L,L,1);
	vec4 H(0,L,L,1);

	// Floor:
	triangles.push_back( Triangle( C, B, A, white ) );
	triangles.push_back( Triangle( C, D, B, white ) );

	// Left wall
	triangles.push_back( Triangle( A, E, C, red) );
	triangles.push_back( Triangle( C, E, G, red) );

	// Right wall
	triangles.push_back( Triangle( F, B, D, dark_green) );
	triangles.push_back( Triangle( H, F, D, dark_green) );

	// Ceiling
	triangles.push_back( Triangle( E, F, G, white ) );
	triangles.push_back( Triangle( F, H, G, white ) );

	// Back wall
	triangles.push_back( Triangle( G, D, C, white) );
	triangles.push_back( Triangle( G, H, D, white) );

	// ---------------------------------------------------------------------------
	// Short block

	A = vec4(290,0,114,1);
	B = vec4(130,0, 65,1);
	C = vec4(240,0,272,1);
	D = vec4( 82,0,225,1);

	E = vec4(290,165,114,1);
	F = vec4(130,165, 65,1);
	G = vec4(240,165,272,1);
	H = vec4( 82,165,225,1);

	// Front
	triangles.push_back( Triangle(E,B,A,dark_green) );
	triangles.push_back( Triangle(E,F,B,dark_green) );

	// BACK
	triangles.push_back( Triangle(H,C,D,white) );
	triangles.push_back( Triangle(H,G,C,white) );

	// LEFT
	triangles.push_back( Triangle(G,E,C,white) );
	triangles.push_back( Triangle(E,A,C,white) );

	// TOP
	triangles.push_back( Triangle(G,F,E,white) );
	triangles.push_back( Triangle(G,H,F,white) );

	// ---------------------------------------------------------------------------
	// Tall block

	A = vec4(423,0,247,1);
	B = vec4(265,0,296,1);
	C = vec4(472,0,406,1);
	D = vec4(314,0,456,1);

	E = vec4(423,330,247,1);
	F = vec4(265,330,296,1);
	G = vec4(472,330,406,1);
	H = vec4(314,330,456,1);

	// Front
	triangles.push_back( Triangle(E,B,A,white) );
	triangles.push_back( Triangle(E,F,B,white) );

	// BACK
	triangles.push_back( Triangle(H,C,D,white) );
	triangles.push_back( Triangle(H,G,C,white) );

	// LEFT
	triangles.push_back( Triangle(G,E,C,white) );
	triangles.push_back( Triangle(E,A,C,white) );

	// TOP
	triangles.push_back( Triangle(G,F,E,white) );
	triangles.push_back( Triangle(G,H,F,white) );


	// ----------------------------------------------
	//Custom model loaded from file
	std::ifstream file("Source/bunny22.obj");
	char buffer[128];
	std::vector<vec3> vertices;
	if (file.is_open())
	{
		//int size = triangles.size();
		std::cout << "Loading"<< std::endl;
		while (!file.eof())
		{
			switch (file.peek())
			{
			case 'v':
				file.get();
				float x, y, z;
				file >> x >> y >> z;
				vertices.push_back(vec3(x, y, z));
				break;
			case 'f':
			{
				file.get();
				int a, b, c;
				file >> a >> b >> c;
				--a; --b; --c;
				// vec4 va = vec4(vertices[a], 1);
				// vec4 vb = vec4(vertices[b], 1);
				// vec4 vc = vec4(vertices[c], 1);
				Triangle triangle(vec4(vertices[a], 1), vec4(vertices[b], 1), vec4(vertices[c], 1), white);
				triangles.push_back(triangle);
				break;
			}
			case '\n':
				file.get();
				break;
			case '#':
			default:
				file.getline(buffer, 128);
				break;
			}
		}
		std::cout << "Loaded custom model" << std::endl;
		file.close();
	}
	else
	{
		std::cout << "Cannot open , aborting" << std::endl;
	}

	// ----------------------------------------------
	// Scale to the volume [-1,1]^3

	for( size_t i=0; i<triangles.size(); ++i )
	{
		triangles[i].v0 *= 2/L;
		triangles[i].v1 *= 2/L;
		triangles[i].v2 *= 2/L;

		triangles[i].v0 -= vec4(1,1,1,1);
		triangles[i].v1 -= vec4(1,1,1,1);
		triangles[i].v2 -= vec4(1,1,1,1);

		triangles[i].v0.x *= -1;
		triangles[i].v1.x *= -1;
		triangles[i].v2.x *= -1;

		triangles[i].v0.y *= -1;
		triangles[i].v1.y *= -1;
		triangles[i].v2.y *= -1;

		triangles[i].v0.w = 1.0;
		triangles[i].v1.w = 1.0;
		triangles[i].v2.w = 1.0;

		triangles[i].ComputeNormal();
	}
}

#endif
