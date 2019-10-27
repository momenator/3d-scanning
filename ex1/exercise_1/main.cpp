#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"

#include "VirtualSensor.h"

using namespace std;

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = height * width;

	// TODO: Determine number of valid faces
	unsigned nFaces = 0;

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << endl;

	// TODO: save vertices
	for (int i = 0; i < nVertices; i++) {
		Vector4f pos = (*(vertices + i)).position;
		Vector4uc col = (*(vertices + i)).color;
		outFile << pos(0) << " " << pos(1) << " " << pos(2) << " " << int(col(0)) << " " << int(col(1)) << " " << int(col(2)) << " " << int(col(3)) << endl;
	}

	// TODO: save valid faces


	// close file
	outFile.close();

	return true;
}

int main()
{
	// cout << "Hello world" << "\n" << endl; // sanity check
	
	// Make sure this path points to the data folder
	std::string filenameIn = "./data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "./mesh/mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		
		const int numPixels = sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight();
		Vertex* vertices = new Vertex[numPixels];

		for (int v = 0; v < sensor.GetDepthImageHeight(); v++) {
			for (int u = 0; u < sensor.GetDepthImageWidth(); u++) {
				float Zc = *(depthMap + u + v);
				// cout << "ZC " << Zc << endl;
				if (Zc == MINF) {
					vertices[u + v].position = Vector4f(MINF, MINF, MINF, MINF);;
					vertices[u + v].color = Vector4uc(0,0,0,0);
				} else {
					// apply backprojection
					float Xc = (u - cX) * Zc / fX;
					float Yc = (v - cY) * Zc / fY;
					Vector4f Xcamera = Vector4f(Xc, Yc, Zc, 1.0);
					Vector4f Xworld = trajectoryInv * Xcamera;
					Vector4uc RGBA = Vector4uc(*(colorMap + u + v + 0), *(colorMap + u + v + 1), *(colorMap + u + v + 2), *(colorMap + u + v + 3));
					vertices[u + v].position = Xworld;
					vertices[u + v].color = RGBA;
				}
			}
		}

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}
