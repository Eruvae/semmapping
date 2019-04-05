#ifndef SEMANTICMAP_H
#define SEMANTICMAP_H

#include "boost_geometry_msgs.h"

namespace semmapping
{

struct SemanticObject
{
  std::string name;
  std::vector<polygon> shapes;
  std::vector<int> shape_certainties;
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

  inline double union_fit(const polygon &newpg, const polygon &refpg)
  {
      multi_polygon un;
      bg::union_(newpg, refpg, un);
      multi_polygon sect;
      bg::intersection(newpg, un, sect);
      return bg::area(sect) / bg::area(un);
  }

public:
  SemanticMap();

  void addEvidence(const std::string &name, const polygon &pg);

  void filterIntersectionThresh(std::set<size_t> &object_list, const polygon &pg);
  int findFittingExistingShape(std::vector<polygon> &shapes, const polygon &pg);
  std::set<size_t> getObjectsInRange(const polygon &pg);
};

}

#endif // SEMANTICMAP_H
