/*
 * ObjectMesh.cpp
 *
 *  Created on: Apr 9, 2014
 *      Author: edgar
 */

#include "ObjectMesh.h"
#include "CsvReader.h"


// --------------------------------------------------- //
//                   VERTEX CLASS                      //
// --------------------------------------------------- //

/**  The default constructor of the Vertex Class */
Vertex::Vertex()
{
  // TODO Auto-generated constructor stub
}

/**  The custom constructor of the Vertex Class */
Vertex::Vertex(const int id, const Point3f P)
{
  id_ = id;
  p_ = P;
}

/**  The default destructor of the Class */
Vertex::~Vertex()
{
  // TODO Auto-generated destructor stub
}


// --------------------------------------------------- //
//                   TRIANGLE CLASS                    //
// --------------------------------------------------- //

/**  The custom constructor of the Triangle Class */
Triangle::Triangle(int id, Vertex V0, Vertex V1, Vertex V2)
{
  id_ = id; v0_ = V0; v1_ = V1; v2_ = V2;
}

/**  The default destructor of the Class */
Triangle::~Triangle()
{
  // TODO Auto-generated destructor stub
}


// --------------------------------------------------- //
//                     RAY CLASS                       //
// --------------------------------------------------- //

/**  The custom constructor of the Ray Class */
Ray::Ray(Point3f P0, Point3f P1) {
  p0_ = P0; p1_ = P1;
}

/**  The default destructor of the Class */
Ray::~Ray()
{
  // TODO Auto-generated destructor stub
}


// --------------------------------------------------- //
//               OBJECT MESH CLASS                     //
// --------------------------------------------------- //

/** The default constructor of the ObjectMesh Class */
ObjectMesh::ObjectMesh() : list_vertex_(0) , map_list_triangles_(0)
{
  id_ = 0;
  num_vertexs_ = 0;
  num_triangles_ = 0;
  vertex_iterator_ = 0;
}

/** The default destructor of the ObjectMesh Class */
ObjectMesh::~ObjectMesh()
{
  // TODO Auto-generated destructor stub
}

// TODO: DOCUMENTATION
void ObjectMesh::incrVertexIterator()
{
  if (vertex_iterator_ < (int)list_vertex_.size()-1) vertex_iterator_++;
}

/** Load a CSV file and fill the object mesh */
void ObjectMesh::loadMesh(const string path) {

  // Create the reader
  CsvReader csvReader(path);

  // Clear previous data
  list_vertex_.clear();
  map_list_triangles_.clear();

  // TODO: check path file to open different formats .ply .stl
  /*
   * if (path == .ply) then readPLY
   * if (path == .stl) then readSTL
   *
   */

  // Read from .ply file
  csvReader.readPLY(list_vertex_, map_list_triangles_);

  // Update mesh attributes
  num_vertexs_ = list_vertex_.size();
  num_triangles_ = map_list_triangles_.size();

  // Read the vertices and construct triangles
  /*for (int i = 0 ; i < num_triangles_; i++)
  {
    Vertex V0(list_vertex_.at(map_list_triangles_.at(i)[0]-1));
    Vertex V1(list_vertex_.at(map_list_triangles_.at(i)[1]-1));
    Vertex V2(list_vertex_.at(map_list_triangles_.at(i)[2]-1));

    Triangle T(i, V0, V1, V2);
  }*/
}
