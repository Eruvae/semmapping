#include "semanticmap.h"
#include <ros/ros.h>
#include <boost/foreach.hpp>

namespace semmapping
{

SemanticMap::SemanticMap()
{

}

void SemanticMap::filterIntersectionThresh(std::set<size_t> &object_list, const polygon &pg)
{
    /*std::remove_if(object_list.begin(), object_list.end(),
    [&, this](size_t x)
    {
        const SemanticObject &obj = objectList.at(x);
        return ref_fit(pg, obj.shape_union) < FIT_THRESH;
    });*/
    ROS_INFO("Filtering objects");
    for (auto it = object_list.begin(); it != object_list.end(); )
    {
        const SemanticObject &obj = objectList.at(*it);
        if (ref_fit(pg, obj.shape_union) < FIT_THRESH)
        {
            object_list.erase(it);
            ROS_INFO("Object removed");
            continue;
        }
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
    ROS_INFO_STREAM("Best fitting shape: " << index << ", fit: " << best_fit);
    return index;
}

int SemanticMap::deleteLeastConsistentShape(SemanticObject &obj)
{
    // TODO
}

void SemanticMap::cleanupShapes(SemanticObject &obj)
{
    for (auto i = obj.shapes.begin(); i < obj.shapes.end() - 1; i++)
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
    ROS_INFO("Calculating union");
    if (obj.shapes.size() < 1)
    {
        ROS_ERROR("Semantic object has no shape");
        return;
    }
    ROS_INFO_STREAM("First shape: " << bg::wkt(obj.shapes[0].shape));
    multi_polygon un;
    un.resize(1);
    un[0] = obj.shapes[0].shape;
    ROS_INFO_STREAM("Shape count: " << obj.shapes.size());
    for (auto i = obj.shapes.begin() + 1; i < obj.shapes.end(); i++)
    {
        //if (i->certainty > MIN_CERTAINTY)
        bg::union_(un, i->shape, un);
    }
    ROS_INFO_STREAM("Union size: " << un.size());
    if (un.size() < 1)
    {
        ROS_ERROR("Union is empty");
        return;
    }
    ROS_INFO_STREAM("Union: " << bg::wkt(un));
    obj.shape_union = un[0];

    ROS_INFO("Calculating bounding box");
    obj.bounding_box = bg::return_envelope<box>(obj.shape_union);
}

void SemanticMap::addEvidence(const std::string &name, const polygon &pg)
{
    ROS_INFO("Collecting existing objects");
    std::set<size_t> existingObjects = getObjectsByNameInRange(name, pg);//getObjectsInRange(pg);
    filterIntersectionThresh(existingObjects, pg);
    if (existingObjects.empty())
    {
        // no object evidence exists yet, create new
        ROS_INFO("Create new object");
        addNewObject(name, pg);
    }
    else
    {
        ROS_INFO_STREAM("Number of fitting objects: " << existingObjects.size());
        // add evidence to each fitting object
        for (size_t id : existingObjects)
        {
            SemanticObject &obj = objectList[id];
            int fittingShapeInd = findFittingExistingShape(obj.shapes, pg);
            if (fittingShapeInd < 0)
            {
                ROS_INFO("No fitting shape found, creating new");
                // not fitting shape found, create new shape
                if (obj.shapes.size() >= MAX_SHAPES)
                {
                    // delete least consistent shape
                    deleteLeastConsistentShape(obj);
                }
                obj.shapes.push_back({pg, 1});
                objectRtree.remove(std::make_pair(obj.bounding_box, fittingShapeInd));
                updateUnion(obj);
                objectRtree.insert(std::make_pair(obj.bounding_box, fittingShapeInd));
            }
            else
            {
                ROS_INFO_STREAM("Fitting shape found: " << fittingShapeInd);
                // add evidence to shape
                // TODO: maybe update shape (union)
                obj.shapes[fittingShapeInd].certainty++;
            }
            obj.exist_certainty++;
        }
    }
}

void SemanticMap::addNewObject(const std::string name, const polygon &initial_shape)
{
    ROS_INFO_STREAM("Initial shape: " << bg::wkt(initial_shape));
    SemanticObject obj;
    obj.name = name;
    obj.shapes.push_back({initial_shape, 1});
    obj.exist_certainty = 1;
    updateUnion(obj);

    ROS_INFO("Adding object to map");
    objectList[next_index] = obj;
    ROS_INFO("Adding object to Rtree");
    ROS_INFO_STREAM("Bounding box: " << bg::wkt(obj.bounding_box));
    rtree_entry ent = {obj.bounding_box, next_index};
    objectRtree.insert(ent);
    //objectRtree.insert(std::make_pair<box, size_t>(obj.bounding_box, next_index));
    ROS_INFO("Succesfully added");
    next_index++;
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

std::set<size_t> SemanticMap::getObjectsByNameInRange(const std::string &name, const polygon &pg)
{
    std::vector<rtree_entry> result_obj;
    std::set<size_t> result;
    objectRtree.query(bgi::intersects(pg), std::back_inserter(result_obj));
    for (rtree_entry val : result_obj)
    {
        const SemanticObject &foundObject = objectList[val.second];
        if (foundObject.name == name && bg::intersects(pg, foundObject.shape_union))
            result.insert(val.second);
    }
    return result;
}

hypermap_msgs::SemanticMap::Ptr SemanticMap::createMapMessage()
{
    hypermap_msgs::SemanticMap::Ptr map(new hypermap_msgs::SemanticMap);

    for (auto val : objectList)
    {
        const SemanticObject &obj = val.second;
        hypermap_msgs::SemanticObject obj_msg;
        obj_msg.id = val.first;
        obj_msg.name = obj.name;
        obj_msg.shape = boostToPolygonMsg(obj.shape_union);
        point centroid;
        bg::centroid(obj.shape_union, centroid);
        obj_msg.position = boostToPointMsg(centroid);

        map->objects.push_back(std::move(obj_msg));
    }

    map->header.frame_id = "map";
    map->header.stamp = ros::Time::now();
    return map;
}

}
