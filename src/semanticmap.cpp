#include "semanticmap.h"

namespace semmapping
{

SemanticMap::SemanticMap()
{

}

void SemanticMap::filterIntersectionThresh(std::set<size_t> &object_list, const polygon &pg)
{
    const double FIT_THRESH = 0.6; // threshold to consider new shape as evidence
    for (auto it = object_list.begin(); it != object_list.end(); )
    {
        SemanticObject &obj = objectList[*it];
        if (ref_fit(pg, obj.shape_union) < FIT_THRESH)
            object_list.erase(it);
        else
            it++;
    }
}

int SemanticMap::findFittingExistingShape(std::vector<polygon> &shapes, const polygon &pg)
{
    const double MIN_FIT = 0.9; // minmal fit for shape to be considered same
    double best_fit = MIN_FIT;
    int index = -1;
    for (int i = 0; i < shapes.size(); i++)
    {
        double shape_fit = union_fit(pg, shapes[i]);
        if (shape_fit > best_fit)
        {
            best_fit = shape_fit;
            index = i;
        }
    }
    return index;
}

void SemanticMap::addEvidence(const std::string &name, const polygon &pg)
{
    std::set<size_t> existingObjects = getObjectsInRange(pg);
    filterIntersectionThresh(existingObjects, pg);
    if (existingObjects.empty())
    {
        // no object evidence exists yet, create new
    }
    else
    {
        // add evidence to each fitting object
        for (size_t id : existingObjects)
        {
            SemanticObject &obj = objectList[id];
            int fittingShapeInd = findFittingExistingShape(obj.shapes, pg);
            if (fittingShapeInd < 0)
            {
                // not fitting shape found, create new shape
            }
            else
            {
                // add evidence to shape
            }
        }
    }
}

std::set<size_t> SemanticMap::getObjectsInRange(const polygon &pg)
{
    std::vector<rtree_entry> result_obj;
    std::set<size_t> result;
    objectRtree.query(bgi::intersects(pg), std::back_inserter(result_obj));
    for (rtree_entry val : result_obj)
    {
        const SemanticObject &foundObject = objectList[val.second];
        if (bg::intersects(pg, foundObject.shape_union))
            result.insert(val.second);
    }
    return result;
}

}
