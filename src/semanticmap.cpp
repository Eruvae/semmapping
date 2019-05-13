#include "semanticmap.h"
#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

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
            it = object_list.erase(it);
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

void SemanticMap::deleteLeastConsistentShape(size_t id)
{
    SemanticObject &obj = objectList.at(id);
    double max_dist = 0;
    auto it = obj.shapes.begin();
    auto toDelete = it;
    for (; it != obj.shapes.end(); it++)
    {
        double dist = bg::distance(obj.centroid_mean, it->centroid);
        if (dist > max_dist)
        {
            max_dist = dist;
            toDelete = it;
        }
    }
    removeFromMean(obj.centroid_mean, toDelete->centroid, obj.shapes.size() - 1);
    obj.shapes.erase(toDelete);
}

/*void SemanticMap::cleanupShapes(SemanticObject &obj)
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
}*/

void SemanticMap::updateUnion(size_t id)
{
    SemanticObject &obj = objectList.at(id);
    ROS_INFO("Calculating union");
    if (obj.shapes.size() < 1)
    {
        ROS_ERROR("Semantic object has no shape");
        return;
    }
    //ROS_INFO_STREAM("First shape: " << bg::wkt(obj.shapes[0].shape));
    multi_polygon un;
    ROS_INFO_STREAM("Shape count: " << obj.shapes.size());

    //size_t best_shape_ind = 0;
    //double best_shape_cert = DBL_MIN;

    //multi_point obj_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    //size_t i = 0;
    for (const UncertainShape &shape : obj.shapes)
    {
        //if (i->certainty > MIN_CERTAINTY)

        //multi_polygon res;
        //bg::union_(un, shape.shape, res);
        //un = std::move(res);

        //un.push_back(shape.shape);

        addToPointCloud(obj_cloud, shape.shape);

        //if (shape.certainty > best_shape_cert)
        //{
        //    best_shape_cert = shape.certainty;
        //    best_shape_ind = i;
        //}
        //i++;
    }
    ROS_INFO_STREAM("Union size: " << un.size());
    //ROS_INFO_STREAM("Union: " << bg::wkt(un));
    //obj.shape_union = un[0];

    //bg::convex_hull(un, obj.shape_union);

    //bg::correct(obj_cloud);
    //bg::convex_hull(obj_cloud, obj.shape_union);
    //bg::correct(obj.shape_union);

    obj.shape_union = computeConvexHullPcl(obj_cloud);

    //obj.shape_union = obj.shapes[best_shape_ind].shape;

    //ROS_INFO("Updating bounding box");
    objectRtree.remove(std::make_pair(obj.bounding_box, id));
    /*std::remove_if(objectRtree.begin(), objectRtree.end(), [id](rtree_entry &ent) // works only with boost >= 1.59
    {
        return ent.second == id;
    });*/
    obj.bounding_box = getSearchBox(obj.shape_union); //bg::return_envelope<box>(obj.shape_union);
    objectRtree.insert(std::make_pair(obj.bounding_box, id));
}

size_t SemanticMap::combineObjects(std::set<size_t> objects)
{
    ROS_INFO_STREAM("Combining " << objects.size() << " objects");
    auto it = objects.begin();
    size_t combinedId = *it;
    it++;
    SemanticObject &combinedObj = objectList.at(combinedId);
    for (; it != objects.end(); it++)
    {
        SemanticObject &toMerge = objectList.at(*it);
        combinedObj.shapes.insert(combinedObj.shapes.end(), toMerge.shapes.begin(), toMerge.shapes.end());
        combinedObj.exist_certainty += toMerge.exist_certainty;
        if (combinedObj.exist_certainty > 20)
            combinedObj.exist_certainty = 20;

        removeObject(*it);
    }
    updateUnion(combinedId);
    return combinedId;
}

// V1
/*void SemanticMap::addEvidence(const std::string &name, const polygon &pg)
{
    //ROS_INFO_STREAM("Collecting existing objects in shape: " << bg::wkt(pg));
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
        size_t objectId = *existingObjects.begin();
        // if more than one object fits, combine objects
        if (existingObjects.size() > 1)
        {
            objectId = combineObjects(existingObjects);
        }
        // add evidence to object
        //for (size_t objectId : existingObjects)
        //{
        SemanticObject &obj = objectList.at(objectId);
        int fittingShapeInd = findFittingExistingShape(obj.shapes, pg);
        if (fittingShapeInd < 0)
        {
            ROS_INFO("No fitting shape found, creating new");
            // not fitting shape found, create new shape
            //if (obj.shapes.size() >= MAX_SHAPES)
            //{
            //    // delete least consistent shape
            //    deleteLeastConsistentShape(objectId);
            //}
            obj.shapes.push_back({pg, 1});
        }
        else
        {
            ROS_INFO_STREAM("Fitting shape found: " << fittingShapeInd);
            // add evidence to shape
            // TODO: maybe update shape (union)
            multi_polygon un;
            UncertainShape &shape = obj.shapes[fittingShapeInd];
            bg::union_(shape.shape, pg, un);
            shape.shape = un[0];
            //ROS_INFO_STREAM("Fitting shape: " << bg::wkt(obj.shapes[fittingShapeInd].shape));
            //ROS_INFO_STREAM("Certainty: " << obj.shapes[fittingShapeInd].certainty);
            shape.certainty++;
            if (shape.certainty > 20)
                shape.certainty = 20;

            //ROS_INFO_STREAM("Certainty increased: " << obj.shapes[fittingShapeInd].certainty);
        }
        updateUnion(objectId);
        obj.exist_certainty++;
        if (obj.exist_certainty > 20)
            obj.exist_certainty = 20;
        //ROS_INFO_STREAM("Exist certainty increased: " << obj.exist_certainty);
        //}
    }
}*/

//V2
void SemanticMap::addEvidence(const std::string &name, const polygon &pg)
{
    box searchArea = getSearchBox(pg);
    std::set<size_t> existingObjects = getObjectsByNameInRange(name, searchArea);
    if (existingObjects.size() == 0)
    {
        ROS_INFO("Create new object");
        addNewObject(name, pg);
    }
    else
    {
        size_t objectId;
        if (existingObjects.size() > 1)
            objectId = combineObjects(existingObjects);
        else
            objectId = *existingObjects.begin();

        SemanticObject &obj = objectList.at(objectId);
        int fittingShapeInd = findFittingExistingShape(obj.shapes, pg);
        if (fittingShapeInd < 0)
        {
            ROS_INFO("No fitting shape found, creating new");
            // not fitting shape found, create new shape
            point shape_centroid;
            bg::centroid(pg, shape_centroid);
            obj.shapes.push_back({pg, shape_centroid, 1});
            addToMean(obj.centroid_mean, shape_centroid, obj.shapes.size());

            if (obj.shapes.size() > MAX_SHAPES) // delete least consistent shape
                deleteLeastConsistentShape(objectId);
        }
        else
        {
            ROS_INFO_STREAM("Fitting shape found: " << fittingShapeInd);
            UncertainShape &shape = obj.shapes[fittingShapeInd];
            // add evidence to shape
            // TODO: maybe update shape (union)
            //multi_polygon un;
            //bg::union_(shape.shape, pg, un);
            //shape.shape = un[0];

            //multi_point obj_cloud;
            //addToPointCloud(obj_cloud, shape.shape);
            //addToPointCloud(obj_cloud, pg);
            //bg::correct(obj_cloud);
            //bg::convex_hull(obj_cloud, shape.shape);
            //bg::correct(shape.shape);

            pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
            addToPointCloud(obj_cloud, shape.shape);
            addToPointCloud(obj_cloud, pg);
            shape.shape = computeConvexHullPcl(obj_cloud);
            removeFromMean(obj.centroid_mean, shape.centroid, obj.shapes.size() - 1);
            bg::centroid(shape.shape, shape.centroid);
            addToMean(obj.centroid_mean, shape.centroid, obj.shapes.size());

            //ROS_INFO_STREAM("Fitting shape: " << bg::wkt(obj.shapes[fittingShapeInd].shape));
            //ROS_INFO_STREAM("Certainty: " << obj.shapes[fittingShapeInd].certainty);
            shape.certainty++;
            if (shape.certainty > 20)
                shape.certainty = 20;

            //ROS_INFO_STREAM("Certainty increased: " << obj.shapes[fittingShapeInd].certainty);
        }
        updateUnion(objectId);
        obj.exist_certainty++;
        if (obj.exist_certainty > 20)
            obj.exist_certainty = 20;
    }
}

void SemanticMap::removeEvidence(const polygon &visibilityArea)
{
    std::set<size_t> existingObjects = getObjectsWithinRange(visibilityArea);
    ROS_INFO_STREAM("Removing evidence for " << existingObjects.size() << " objects");
    for (size_t id : existingObjects)
    {
        SemanticObject &obj = objectList.at(id);
        //if (bg::within(obj.shape_union, visibilityArea)) // whole union is visible
        //{
        obj.exist_certainty -= 0.2;
        ROS_INFO_STREAM("Object " << id << " certainty: " << obj.exist_certainty);

        if (obj.exist_certainty < -5)
        {
            removeObject(id);
            ROS_INFO_STREAM("Object " << id << " removed");
        }
        /*}
        else
        {
            for (auto it = obj.shapes.begin(); it != obj.shapes.end();)
            {
                UncertainShape &shape = *it;
                if (bg::within(shape.shape, visibilityArea)) // whole union is visible
                {
                    shape.certainty -= 0.5;
                    if (shape.certainty < -5)
                    {
                        it = obj.shapes.erase(it);
                        ROS_INFO_STREAM("Shape from " << id << " removed");
                        continue;
                    }
                }
                it++;
            }
        }*/
    }
}

void SemanticMap::addNewObject(const std::string name, const polygon &initial_shape)
{
    ROS_INFO_STREAM("Initial shape: " << bg::wkt(initial_shape));
    SemanticObject obj;
    obj.name = name;
    point shape_centroid;
    bg::centroid(initial_shape, shape_centroid);
    obj.shapes.push_back({initial_shape, shape_centroid, 1});
    obj.exist_certainty = 1;
    obj.shape_union = initial_shape;
    obj.centroid_mean = shape_centroid;
    obj.bounding_box = getSearchBox(obj.shape_union); //bg::return_envelope<box>(obj.shape_union);

    ROS_INFO("Adding object to map");
    objectList[next_index] = obj;
    ROS_INFO_STREAM("Adding object to Rtree, size: " << objectRtree.size());
    ROS_INFO_STREAM("Bounding box: " << bg::wkt(obj.bounding_box) << "; index: " << next_index);
    rtree_entry ent = {obj.bounding_box, next_index};
    objectRtree.insert(ent);
    //objectRtree.insert(std::make_pair<box, size_t>(obj.bounding_box, next_index));
    ROS_INFO_STREAM("Succesfully added, size: " << objectRtree.size());
    next_index++;
}

void SemanticMap::removeObject(size_t id)
{
    SemanticObject &obj = objectList.at(id);
    objectRtree.remove(std::make_pair(obj.bounding_box, id));
    objectList.erase(id);
}

std::set<size_t> SemanticMap::getObjectsInRange(const polygon &pg)
{
    std::vector<rtree_entry> result_obj;
    std::set<size_t> result;
    objectRtree.query(bgi::intersects(pg), std::back_inserter(result_obj));
    for (rtree_entry val : result_obj)
    {
        const SemanticObject &foundObject = objectList.at(val.second);
        if (bg::intersects(pg, foundObject.shape_union))
            result.insert(val.second);
    }
    return result;
}

std::set<size_t> SemanticMap::getObjectsWithinRange(const polygon &pg)
{
    std::vector<rtree_entry> result_obj;
    std::set<size_t> result;
    objectRtree.query(bgi::intersects(pg), std::back_inserter(result_obj));
    for (rtree_entry val : result_obj)
    {
        const SemanticObject &foundObject = objectList.at(val.second);
        if (bg::within(foundObject.shape_union, pg))
            result.insert(val.second);
    }
    return result;
}

std::set<size_t> SemanticMap::getObjectsByNameInRange(const std::string &name, const polygon &pg)
{
    ROS_INFO_STREAM("List size: " << objectList.size() << "; Rtree: " << objectRtree.size());
    std::vector<rtree_entry> result_obj;
    std::set<size_t> result;
    objectRtree.query(bgi::intersects(pg), std::back_inserter(result_obj));
    for (rtree_entry val : result_obj)
    {
        // ROS_INFO_STREAM("Val second: " << val.second);
        const SemanticObject &foundObject = objectList.at(val.second);
        if (foundObject.name == name && bg::intersects(pg, foundObject.shape_union))
            result.insert(val.second);
    }
    return result;
}

std::set<size_t> SemanticMap::getObjectsByNameInRange(const std::string &name, const box &bx)
{
    ROS_INFO_STREAM("List size: " << objectList.size() << "; Rtree: " << objectRtree.size());
    std::vector<rtree_entry> result_obj;
    std::set<size_t> result;
    objectRtree.query(bgi::intersects(bx), std::back_inserter(result_obj));
    for (rtree_entry val : result_obj)
    {
        // ROS_INFO_STREAM("Val second: " << val.second);
        const SemanticObject &foundObject = objectList.at(val.second);
        if (foundObject.name == name/* && bg::intersects(bx, foundObject.shape_union)*/)
            result.insert(val.second);
    }
    return result;
}

hypermap_msgs::SemanticMap::Ptr SemanticMap::createMapMessage()
{
    hypermap_msgs::SemanticMap::Ptr map(new hypermap_msgs::SemanticMap);

    //ROS_INFO_STREAM("Creating message, objects: " << objectList.size());
    for (auto val : objectList)
    {
        const SemanticObject &obj = val.second;
        if (obj.exist_certainty > 1)
        {
            hypermap_msgs::SemanticObject obj_msg;
            obj_msg.id = val.first;
            obj_msg.name = obj.name;
            //ROS_INFO_STREAM("Converting shape: " << bg::wkt(obj.shape_union));
            obj_msg.shape = boostToPolygonMsg(obj.shape_union);
            point centroid;
            //ROS_INFO("Calculating centroid");
            bg::centroid(obj.shape_union, centroid);
            //ROS_INFO_STREAM("Centroid: " << bg::wkt(centroid));
            obj_msg.position = boostToPointMsg(centroid);

            map->objects.push_back(std::move(obj_msg));
        }
    }

    map->header.frame_id = "map";
    map->header.stamp = ros::Time::now();
    //ROS_INFO("Created message successfully");
    return map;
}

bool SemanticMap::writeMapData(std::ostream &output)
{
    YAML::Node map;
    for(const auto &map_entry : objectList)
    {
        const SemanticObject &obj = map_entry.second;
        YAML::Node n;
        n["name"] = obj.name;
        n["exist_certainty"] = obj.exist_certainty;
        /*for (const point &p : obj.shape.outer())
        {
            YAML::Node yp;
            yp.push_back(p.x());
            yp.push_back(p.y());
            yp.SetStyle(YAML::EmitterStyle::Flow);
            n["shape"].push_back(yp);
        }
        n["shape"].SetStyle(YAML::EmitterStyle::Flow);*/
        for (const UncertainShape &shape : obj.shapes)
        {
            std::ostringstream sh;
            sh << bg::wkt(shape.shape);
            YAML::Node us;
            us["shape"] = sh.str();
            us["certainty"] = shape.certainty;
            n["shapes"].push_back(us);
        }
        std::ostringstream sh;
        sh << bg::wkt(obj.shape_union);
        n["shape_union"] = sh.str();
        map.push_back(n);
    }
    output << map;
    return true;
}

}
