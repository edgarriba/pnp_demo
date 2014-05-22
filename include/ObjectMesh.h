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
//                  VERTEX CLASS                       //
// --------------------------------------------------- //

class Vertex {
public:
    /**
    * The default constructor of the Vertex Class.
    *
    * @param
    * @return
    */
    Vertex();

   /**
    * The custom constructor of the Triangle Class.
    *
    * @param id - The identification number.
    * @param P - The 3D coordinates of the vertex.
    *
    */
   explicit Vertex(const int id, const cv::Point3f P);

   /** The default destructor of the Class */
   virtual ~Vertex();

   int getId() const { return id_; }
   cv::Point3f getPoint() const { return p_; }


private:
  /** The identifier number of the vertex */
  int id_;
  /** The 3D coordinates of the vertex */
  cv::Point3f p_;
};



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
  explicit Triangle(int id, Vertex V0, Vertex V1, Vertex V2);

  /** The default destructor of the Class */
  virtual ~Triangle();

  /**
   * Set the identification number of the triangle
   *
   * @param id - The identification number.
   * @return
   *
   */
  void setTriangleID(int id) { id_ = id; }
  Vertex getV0() { return v0_; }
  Vertex getV1() { return v1_; }
  Vertex getV2() { return v2_; }

private:
  /** The identifier number of the triangle */
  int id_;
  /** The three vertices that composes the triangle */
  Vertex v0_, v1_, v2_;
};

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
   * Set the identification number of the object mesh
   *
   * @param id - The identification number.
   * @return
   *
   */
  void setMeshID(int id) { id_ = id; }

  /**
   * Reset the vertex list iterator
   *
   * @param
   * @return
   *
   */
  void resetVertexIterator() { vertex_iterator_ = 0; }

  /**
   * Increment one position the vertex iterator
   *
   * @param
   * @return
   *
   */
  void incrVertexIterator();

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
  Vertex getVertex(int pos) const { return list_vertex_.at(pos); }

  /**
    * Get the current vertex position in list_vertex_
    *
    * @param
    * @return - The current position in list_vertex
    *
    */
  int getVertexIter() const { return vertex_iterator_; }
  std::vector<std::vector<int> > getTrianglesList() const { return map_list_triangles_; }

private:
  /** The identification number of the mesh */
  int id_;
  /** The current number of vertices in the mesh */
  int num_vertexs_;
  /** The current number of triangles in the mesh */
  int num_triangles_;
  /** The current position of vertex list */
  int vertex_iterator_;
  /* The list of triangles of the mesh */
  std::vector<Vertex> list_vertex_;
  /* The list of triangles of the mesh */
  std::vector<std::vector<int> > map_list_triangles_;
};

#endif /* OBJECTMESH_H_ */
