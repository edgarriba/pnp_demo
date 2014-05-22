/*
 * ObjectMesh.h
 *
 *  Created on: Apr 9, 2014
 *      Author: edgar
 */

#ifndef OBJECTMESH_H_
#define OBJECTMESH_H_

#include <iostream>
#include <opencv2/core.hpp>


// --------------------------------------------------- //
//                 TRIANGLE CLASS                      //
// --------------------------------------------------- //

class Triangle {
public:
  /**
   * The custom constructor of the Triangle Class.
   *
   * @param id - The identification number.
   * @param V0 - The first point of the triangle.
   * @param V1 - The second point of the triangle.
   * @param V2 - The third point of the triangle.
   * @return
   *
   */
  explicit Triangle(int id, cv::Point3f V0, cv::Point3f V1, cv::Point3f V2);

  /** The default destructor of the Class */
  virtual ~Triangle();


  cv::Point3f getV0() const { return v0_; }
  cv::Point3f getV1() const { return v1_; }
  cv::Point3f getV2() const { return v2_; }

private:
  /** The identifier number of the triangle */
  int id_;
  /** The three vertices that composes the triangle */
  cv::Point3f v0_, v1_, v2_;
};


// --------------------------------------------------- //
//                     RAY CLASS                       //
// --------------------------------------------------- //

class Ray {
public:
  explicit Ray(cv::Point3f P0, cv::Point3f P1);
  virtual ~Ray();
  cv::Point3f getP0() { return p0_; }
  cv::Point3f getP1() { return p1_; }
private:
  cv::Point3f p0_, p1_;
};


// --------------------------------------------------- //
//                OBJECT MESH CLASS                    //
// --------------------------------------------------- //

class ObjectMesh
{
public:
  /**
  * The default constructor of the ObjectMesh Class.
  *
  * @param
  * @return
  *
  */
  ObjectMesh();

  /** The default destructor of the Class */
  virtual ~ObjectMesh();

  /**
   * Load a CSV file and fill the object mesh
   *
   * @param path_file - The path into the computer of the mesh file.
   * @return
   *
   */
  void loadMesh(const std::string path_file);

   /**
    * Get the current number of vertices
    *
    * @param
    * @return - The current number of vertices
    *
    */
  int getNumVertices() const { return num_vertexs_; }

  /**
    * Get a vertex form the list given a position
    *
    * @param
    * @return - A vertex given a postion in the list
    *
    */
  cv::Point3f getVertex(int pos) const { return list_vertex_[pos]; }

  std::vector<std::vector<int> > getTrianglesList() const { return list_triangles_; }

private:
  /** The identification number of the mesh */
  int id_;
  /** The current number of vertices in the mesh */
  int num_vertexs_;
  /** The current number of triangles in the mesh */
  int num_triangles_;
  /* The list of triangles of the mesh */
  std::vector<cv::Point3f> list_vertex_;
  /* The list of triangles of the mesh */
  std::vector<std::vector<int> > list_triangles_;
};

#endif /* OBJECTMESH_H_ */
