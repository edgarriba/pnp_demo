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
