#ifndef _GRAVIT_PLOT_H_         
#define _GRAVIT_PLOT_H_
#include <vector>
#include <stdio.h>

class VisitAdapter{
protected:
	struct GravitProgramConfig
	{   
	    // Camera Params
	    double focalPoint[3];
	    double upVector[3];
	    double view_direction[3];
	    double fov;
	    double zoom;
	    int filmSize[2];

	    //Tracing Parms
	    int maxDepth;
	    int raySamples;
	    double windowJitterSize;
	    unsigned char backgroundColor[3];
	    bool enableShadows;

	    // Parms from data
	    double dataBoundingBox[6];

	    // Trace Parms
	    int traceScheduler;

	    // Light Parms
	    float lightBoost;
	};

	struct GravitProgramConfig gravitProgramConfig;
	int (*loadBlockFunc)(void *, int, double ** , int& , int ** , int& );
	void * loadBlockObj;

	std::vector<int> instanceIds;
	std::vector<int> mpiInstanceIds;
	int totalInstances;

public:

	struct RayTraceProperties
	{
		int maxDepth;
		int raySamples;
		double windowJitterSize;
		unsigned char backgroundColor[3];
	};

	VisitAdapter();
	~VisitAdapter();

	void SetVisitProcessBlockFunc(void * obj, int(*loadBlock)(void *, int, double ** , int& , int ** , int& )) {loadBlockObj = obj; loadBlockFunc = loadBlock;}

	void SetLight(int numberOfLights, int * lighttypes, double * lightDirection, unsigned char * color, double * lightIntensity, bool enableShadows, float lightBoost, float lightDistance);

        void SetRayTraceProperties(RayTraceProperties properties);

        void SetTraceMode(int mode){ gravitProgramConfig.traceScheduler = mode;}

        void RegisterDomain(int instanceId, int mpiInstanceId);

        void SetTotalInstances(int totalInstances);

        void ChangeMaterial(int meshId, int material, double * materialProp);

        void ResetMeshAndInstance();

        void Draw(unsigned char * input);

	void SetData(double * points, int numPoints, int * edges, int numEdges, int Material, double * materialProp);

	void SetBoundingBoxHolder(double * points, int numPoints, int * edges, int numEdges, int Material, double * materialProp);

	void SetCamera(int* imageSize,  double * focalPoint, double * upVector, double * viewDirection, double zoom, double fov);
};

#endif
