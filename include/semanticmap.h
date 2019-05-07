#ifndef SEMANTICMAP_H
#define SEMANTICMAP_H

#include "boost_geometry_msgs.h"
#include <hypermap_msgs/SemanticMap.h>

namespace semmapping
{

const size_t MAX_SHAPES = 10; // maximum number of differnt shapes kept in object
const int MIN_CERTAINTY = 10;
const double FIT_THRESH = 0.2; // threshold to consider new shape as evidence
const double MIN_FIT = 0.8; // minmal fit for shape to be considered same

struct UncertainShape
{
    polygon shape;
    double certainty;
};

struct SemanticObject
{
  std::string name;
  std::vector<UncertainShape> shapes;
  double exist_certainty;
  polygon shape_union;
  box bounding_box;
  //std::vector<std::string> tags;
  //std::vector<double> confidence;
};

class SemanticMap
{
  typedef std::pair<box, size_t> rtree_entry;

  size_t next_index = 0;
  bgi::rtree< rtree_entry, bgi::rstar<16> > objectRtree;
  std::map<size_t, SemanticObject> objectList;

  inline static double ref_fit(const polygon &newpg, const polygon &refpg)
  {
      multi_polygon sect;
      bg::intersection(newpg, refpg, sect);
      return bg::area(sect) / bg::area(refpg);
  }

  inline static double union_fit(const polygon &a, const polygon &b)
  {
      multi_polygon un;
      bg::union_(a, b, un);
      multi_polygon sect;
      bg::intersection(a, b, sect);
      return bg::area(sect) / bg::area(un);
  }

  inline static box getSearchBox(const polygon &pg)
  {
      point centroid;
      bg::centroid(pg, centroid);
      //double searchRadius = bg::perimeter(pg) / 8;
      //double searchRadius = std::sqrt(bg::area(pg));
      //if (searchRadius < 1)
      //    searchRadius = 1;
      double searchRadius = 0.5;

      return box(point(centroid.x() - searchRadius, centroid.y() - searchRadius), point(centroid.x() + searchRadius, centroid.y() + searchRadius));
  }

public:
  SemanticMap();

  void addEvidence(const std::string &name, const polygon &pg);
  void removeEvidence(const polygon &visibilityArea);

  //int deleteLeastConsistentShape(size_t id);
  //void cleanupShapes(SemanticObject &obj);
  void updateUnion(size_t id);
  void filterIntersectionThresh(std::set<size_t> &object_list, const polygon &pg);
  int findFittingExistingShape(std::vector<UncertainShape> &shapes, const polygon &pg);
  void addNewObject(const std::string name, const polygon &initial_shape);
  void removeObject(size_t id);
  size_t combineObjects(std::set<size_t> objects);
  std::set<size_t> getObjectsInRange(const polygon &pg);
  std::set<size_t> getObjectsWithinRange(const polygon &pg);
  std::set<size_t> getObjectsByNameInRange(const std::string &name, const polygon &pg);
  std::set<size_t> getObjectsByNameInRange(const std::string &name, const box &bx);

  hypermap_msgs::SemanticMap::Ptr createMapMessage();

  bool writeMapData(std::ostream &output);
};

}

#endif // SEMANTICMAP_H
