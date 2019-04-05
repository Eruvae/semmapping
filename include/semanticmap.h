#ifndef SEMANTICMAP_H
#define SEMANTICMAP_H

#include "boost_geometry_msgs.h"

namespace semmapping
{

const size_t MAX_SHAPES = 10; // maximum number of differnt shapes kept in object
const double FIT_THRESH = 0.6; // threshold to consider new shape as evidence
const double MIN_FIT = 0.9; // minmal fit for shape to be considered same

struct UncertainShape
{
    polygon shape;
    int certainty;
};

struct SemanticObject
{
  std::string name;
  std::vector<UncertainShape> shapes;
  int exist_certainty;
  polygon shape_union;
  box bounding_box;
  std::vector<std::string> tags;
  std::vector<double> confidence;
};

class SemanticMap
{
  typedef std::pair<box, size_t> rtree_entry;

  size_t next_index;
  bgi::rtree< rtree_entry, bgi::rstar<16> > objectRtree;
  std::map<size_t, SemanticObject> objectList;

  inline double ref_fit(const polygon &newpg, const polygon &refpg)
  {
      multi_polygon sect;
      bg::intersection(newpg, refpg, sect);
      return bg::area(sect) / bg::area(refpg);
  }

  inline double union_fit(const polygon &a, const polygon &b)
  {
      multi_polygon un;
      bg::union_(a, b, un);
      multi_polygon sect;
      bg::intersection(a, b, sect);
      return bg::area(sect) / bg::area(un);
  }

public:
  SemanticMap();

  void addEvidence(const std::string &name, const polygon &pg);

  int deleteLeastConsistentShape(SemanticObject &obj);
  void cleanupShapes(SemanticObject &obj);
  void updateUnion(SemanticObject &obj);
  void filterIntersectionThresh(std::set<size_t> &object_list, const polygon &pg);
  int findFittingExistingShape(std::vector<UncertainShape> &shapes, const polygon &pg);
  std::set<size_t> getObjectsInRange(const polygon &pg);
};

}

#endif // SEMANTICMAP_H
