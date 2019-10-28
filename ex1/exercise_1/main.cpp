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

bool hasInfPosition(Vector4f posA, Vector4f posB, Vector4f posC)
{
	// only need to check X, if X is MINF, the rest is invalid or also MINF
	return posA(0) == MINF || posB(0) == MINF || posC(0) == MINF;
}

bool areEdgesValid(Vector4f posA, Vector4f posB, Vector4f posC, float threshold)
{
	Vector3f pos3A = Vector3f(posA(0), posA(1), posA(2));
	Vector3f pos3B = Vector3f(posB(0), posB(1), posB(2));
	Vector3f pos3C = Vector3f(posC(0), posC(1), posC(2));

	// check A -> B
	float norm1 = (pos3B - pos3A).norm();
	bool norm1Valid = norm1 > 0 && norm1 < threshold;

	// check B -> C
	float norm2 = (pos3C - pos3B).norm();
	bool norm2Valid = norm2 > 0 && norm2 < threshold;

	// check C -> A
	float norm3 = (pos3A - pos3C).norm();
	bool norm3Valid = norm3 > 0 && norm3 < threshold;

	// if (norm1Valid || norm2Valid || norm3Valid) {
	// 	cout << "NORM VALID " << endl;
	// 	cout << "POS:: " << pos3A << " " << pos3B << " " << pos3C << endl;
	// 	cout << "NORM:: " << norm1 << " " << norm2 << " " << norm3 << endl;
	// }

	return norm1Valid && norm2Valid && norm3Valid;
}

int getNumValidFaces(unsigned int height, unsigned int width)
{
	return 2 * (height - 1) * (width - 1);
}

int getNumEdges(unsigned int height, unsigned int width)
{
	return ((height - 1) * (width)) + ((width - 1) * (height)) + ((height - 1) * (width - 1));
}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.5f; // now is 50cm. 1cm - original 0.01f

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
	unsigned int nFaces = getNumValidFaces(height, width);

	unsigned int nEdges = getNumEdges(height, width);


	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " " << nEdges << endl;

	// TODO: save vertices
	for (int i = 0; i < nVertices; i++) {
		Vector4f pos = (*(vertices + i)).position;
		Vector4uc col = (*(vertices + i)).color;
		if (pos(0) == MINF) {
			outFile << "0 0 0 0 0 0 0" << endl;
 		} else {
			outFile << pos(0) << " " << pos(1) << " " << pos(2) << " " << int(col(0)) << " " << int(col(1)) << " " << int(col(2)) << " " << int(col(3)) << endl;
		}
	}
	
	outFile << "# list of faces" << endl;

	// triangulate - get all triplets
	for (int i = 0; i < height - 1; i++) {
		for (int j = 0; j < width - 1; j++) {
			// there are 2 triangles in every 'square', given 4 points tl, tr, bl, br
			// tl = i * height, bl = i * height + width, tr = i * height + 1, br = i * height + width + 1
			// this ordering ensures that the orientaton is anti-clockwise
			// [tl, bl, tr] - top
			// [bl, br, tr] - bottom

			int tl = (i * height) + j;
			int bl = (i * height) + width + j;
			int tr = tl + 1;
			int br = bl + 1;

			Vector4f tlPos = vertices[tl].position;
			Vector4f blPos = vertices[bl].position;
			Vector4f trPos = vertices[tr].position;
			Vector4f brPos = vertices[br].position;

			// check if faces are valid, how?
			// leave it out if any of the points if MINF
			// then check if the each edge is less than threshold
			
			// TODO: save valid faces
			// check top triangle
			if (!hasInfPosition(tlPos, blPos, trPos) && areEdgesValid(tlPos, blPos, trPos, edgeThreshold)) {
				outFile << "3 " << tl << " " << bl << " " << tr << endl;
			}

			// check bottom triangle
			if (!hasInfPosition(blPos, brPos, trPos) && areEdgesValid(blPos, brPos, trPos, edgeThreshold)) {
				outFile << "3 " << bl << " " << br << " " << tr << endl;
			}
		}
	}

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
		const int height = sensor.GetDepthImageHeight();
		const int width = sensor.GetDepthImageWidth();
		const int numPixels = height * width;
		Vertex* vertices = new Vertex[numPixels];

		for (int v = 0; v < height; v++) {
			for (int u = 0; u < width; u++) {
				int idx = (v * height) + u;
				float Zc = depthMap[idx];
				if (Zc == MINF) {
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);;
					vertices[idx].color = Vector4uc(0,0,0,0);
				} else {
					// apply backprojection
					// pixel to camera coordinates to camera space (task 1a)
					float Xc = (u - cX) * Zc / fX;
					float Yc = (v - cY) * Zc / fY;

					// (task 1b optional)
					Vector4f Xcamera = depthExtrinsicsInv * Vector4f(Xc, Yc, Zc, 1.0);

					// camera to world space (task 1b)
					Vector4f Xworld = trajectoryInv * Xcamera;
					
					// cout << " xworld " << Xworld << endl;
					Vector4uc RGBA = Vector4uc(*(colorMap + u + v + 0), *(colorMap + u + v + 1), *(colorMap + u + v + 2), *(colorMap + u + v + 3));
					vertices[idx].position = Xworld;
					vertices[idx].color = RGBA;
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
