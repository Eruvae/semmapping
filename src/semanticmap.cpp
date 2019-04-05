#include "semanticmap.h"

namespace semmapping
{

SemanticMap::SemanticMap()
{

}

void SemanticMap::filterIntersectionThresh(std::set<size_t> &object_list, const polygon &pg)
{
    //std::remove_if(object_list.begin(), object_list.end(), [](size_t x){return ref_fit(pg, objectList[x].shape_union) < FIT_THRESH});
    for (auto it = object_list.begin(); it != object_list.end(); )
    {
        SemanticObject &obj = objectList[*it];
        if (ref_fit(pg, obj.shape_union) < FIT_THRESH)
            object_list.erase(it);
        else
            it++;
    }
}

int SemanticMap::findFittingExistingShape(std::vector<UncertainShape> &shapes, const polygon &pg)
{
    double best_fit = MIN_FIT;
    int index = -1;
    for (int i = 0; i < shapes.size(); i++)
    {
        double shape_fit = union_fit(pg, shapes[i].shape);
        if (shape_fit >= best_fit)
        {
            best_fit = shape_fit;
            index = i;
        }
    }
    return index;
}

int SemanticMap::deleteLeastConsistentShape(SemanticObject &obj)
{

}

void SemanticMap::cleanupShapes(SemanticObject &obj)
{
    for (auto i = obj.shapes.begin(); i != obj.shapes.end() - 1; i++)
    {
        for (auto j = i + 1; j != obj.shapes.end(); j++)
        {
            double shape_fit = union_fit(i->shape, j->shape);
            if (shape_fit >= MIN_FIT)
            {
                multi_polygon un;
                bg::union_(i->shape, j->shape, un);
                i->shape = un[0];
                i->certainty += j->certainty;
                obj.shapes.erase(j);
            }
            else
            {
                j++;
            }
        }
    }
    updateUnion(obj);
}

void SemanticMap::updateUnion(SemanticObject &obj)
{
    multi_polygon un;
    for (auto i = obj.shapes.begin(); i != obj.shapes.end() - 1; i++)
    {
        bg::union_(un, i->shape, un);
    }
    obj.shape_union = un[0];
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
                if (obj.shapes.size() >= MAX_SHAPES)
                {
                    // delete least consistent shape
                }

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
