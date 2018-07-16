//#include <tbb/tbb.h>
#include <stdio.h>
#include <math.h>
#include <gmtl/gmtl.h>
#include <imageio.h>
#include <iostream>
#include <world.h>
#include <standard.h>
#include "../lights/pointlight.h"

#include <parsers/ass_parser.h>

#include <reporter.h>

#define WIN32_LEAN_AND_MEAN
#define AMBIENT_INT 0.01f
#include <Windows.h>
#include <algorithm>

#include <main.h>

int g_RenderMaxDepth = 12;

extern int g_pixel_samples;

World* ReadFromFile(const char* filename)
{
    World* world;
    if (!ReadAssFile(filename, world))
    {
        Error("Could not read file: %s", filename);
        return NULL;
    }
    return world;
}

gmtl::Vec3f refractedDirection(float ni, float nt, const gmtl::Vec3f& V, const gmtl::Vec3f& N)
{
	gmtl::Vec3f T;
	float eta;

	eta = ni / nt;
	float c1 = -dot(V, N);
	float c2_op = 1.0f - eta*eta*(1.0f - c1*c1);
	if (c2_op < 0.0f)
		return gmtl::Vec3f(0.0f);

	float c2 = sqrt(c2_op);
	T = eta*V + (eta*c1 - c2)*N;

	return T;
}

gmtl::Vec3f transmittedDirection(bool& entering, const gmtl::Vec3f& N, const gmtl::Vec3f& V, const Standard* mat)
{
	gmtl::Vec3f normal_refraction = N;
	bool exitingObject = dot(N, -V) < 0.0f;
	float ni = 0.0f;
	float nt = 0.0f;

	if (exitingObject)
	{
		ni = mat->refractionIndex;
		nt = 1.0003f; // air refraction index
		normal_refraction = -normal_refraction;
	}
	else
	{
		ni = 1.0003f; // air refraction index
		nt = mat->refractionIndex;
	}

	gmtl::Vec3f T = refractedDirection(ni, nt, V, normal_refraction);

	entering = !exitingObject;
	return T;
}

Spectrum traceRay(World* world, gmtl::Rayf& ray, int depth = 0) 
{
	if (depth < 0) 
	{
		return Spectrum(0.0f, 0.0f, 0.0f);
	}

	IntersectInfo info;
	float dist = world->intersect(info, ray);

	if (info.objectID == InvalidObjectID) {
		return Spectrum(0.f, 0.f, 0.f);
	}
	else {
		Standard *mat = (Standard*)info.material;
		Vector3f normal = info.normal;
		normalize(normal);
		float kshi = mat->Kshi;

		// constante difusa
		Spectrum kd = mat->Kd.GetColor(info);
		Spectrum cd;

		// constante especular
		Spectrum ks = mat->Ks.GetColor(info);
		Spectrum cs;

		// constante reflexion
		Spectrum reflexion = mat->Kr.GetColor(info);

		// constante refraccion
		Spectrum refraction = mat->Kt.GetColor(info);

		// ambiente
		Spectrum ambient = mat->Ka_color.GetColor(info) * AMBIENT_INT;

		// vector v
		Vector3f v = -ray.getDir();
		normalize(v);

		// recorremos las luces.
		for each(auto light in world->mLights) {
			PointLight *pl = (PointLight *)light;
			Vector3f lightDir = pl->getWorldPosition() - info.position;
			float factorDistance = 1 / gmtl::lengthSquared(lightDir);

			normalize(lightDir);

			IntersectInfo shadowInfo;
			world->intersect(shadowInfo, gmtl::Rayf(info.position + lightDir * 0.01f, lightDir));

			if ((shadowInfo.objectID == InvalidObjectID)) {
				// difuso
				float dotProduct = gmtl::dot(normal, lightDir);
				float cosVal;

				if (dotProduct > 0.0) {
					cosVal = dotProduct;
				}
				else {
					cosVal = 0.0;
				}

				cd += kd * pl->mIntensity * factorDistance * cosVal;

				// especular
				Vector3f r = -gmtl::reflect(r, lightDir, normal);

				cs += ks * pl->mIntensity * factorDistance * pow(max(dot(r, v), 0.0f), kshi);
			}
		}

		// reflexion
		Vector3f reflectVec = -gmtl::reflect(reflectVec, v, normal);
		gmtl::Rayf reflectRay(info.position + reflectVec * 0.01f, reflectVec);
		Spectrum cr = traceRay(world, reflectRay, depth-1) * reflexion;

		Spectrum ct = Spectrum(0.0f, 0.0f, 0.0f);

		// refraccion
		if (mat->refractionIndex != 0) {
			bool entering;
			gmtl::Vec3f t = transmittedDirection(entering, info.normal, info.ray.getDir(), mat);
			gmtl::Rayf refractionRay = gmtl::Rayf(info.position + t * 0.01f, t);
			ct = traceRay(world, refractionRay, depth-1) * refraction;
		}

		// suma todos los componentes
		return cs + cd + ambient + cr + ct;
	}
}

void render_image(World* world, unsigned int dimX, unsigned int dimY, float* image, float* alpha)
{
	for (int i = 0; i < dimX; i++) {
		for (int j = 0; j < dimY; j++) {
			gmtl::Rayf ray;
			ray = world->getCamera()->generateRay((float)i, (float) j);
			
			Spectrum s = traceRay(world, ray, 2);

			image[((j*dimX + i) * 3)] = s[0];
			image[((j*dimX + i) * 3) + 1] = s[1];
			image[((j*dimX + i) * 3) + 2] = s[2];
			alpha[j*dimX + i] = 1.0f;
		}
	}
}

unsigned int g_intersectTriangleCalls;
extern "C"
{
    __declspec(dllexport) void renderSceneFromFile(float*& image, float*& alpha, World*& world, const char* filename)
    {
        google::InitGoogleLogging("rendering.dll");
        FLAGS_logtostderr = 1;

        g_intersectTriangleCalls = 0;

        // Create world from file
        world = ReadFromFile(filename);
        if (!world)
        {
            fprintf(stderr, "Error reading file %s. Press enter to exit", filename);
            getc(stdin);
            return;
        }
        INITREPORTER("report.ma", world);
        unsigned int dimX = world->getCamera()->getResolution()[0];
        unsigned int dimY = world->getCamera()->getResolution()[1];

        image = new float[dimX*dimY * 3];
        alpha = new float[dimX*dimY];


        // Compute pixel values
        clock_t tStart = clock();
        render_image(world, dimX, dimY, image, alpha);

        clock_t tEnd = clock();
        LOG(INFO) << "Time taken: " << (double)(tEnd - tStart) / CLOCKS_PER_SEC << "s";
        LOG(INFO) << "Triangles intersected: " << g_intersectTriangleCalls;

        google::ShutdownGoogleLogging();
    }

    __declspec(dllexport) void WriteImg(const char* name, float *pixels, float *alpha, int xRes,
        int yRes, int totalXRes, int totalYRes, int xOffset, int yOffset)
    {
        WriteImage(name, pixels, alpha, xRes, yRes, totalXRes, totalYRes, xOffset, yOffset);
    }
}

// dllmain.cpp : Defines the entry point for the DLL application.

BOOL APIENTRY DllMain(HMODULE hModule,
    DWORD  ul_reason_for_call,
    LPVOID lpReserved
)
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}
