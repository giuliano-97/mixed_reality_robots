// Standard ros includes
#include <sstream>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// move_group and collision object registration
#include <geometric_shapes/shape_operations.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// Collision object transfer msgs and actions
#include <panda_unity_simulation/BoundingBox.h>
#include <panda_unity_simulation/SpatialMappingMesh.h>
#include <panda_unity_simulation/RegisterCollisionObjectsAction.h>

using namespace Eigen;
using namespace panda_unity_simulation;

class ManipObstaclesManager
{
public:
    // Ctor
    ManipObstaclesManager(const std::string& actionName) :
        server_(nh, actionName, boost::bind(&ManipObstaclesManager::registerCollisionObjs_CB, this, _1), false),
        actionName_(actionName)
    {
        server_.start();
    }

    // Default dtor - nothing special
    ~ManipObstaclesManager()
    {
    }

    void pushFeedback(){
        std::stringstream ss;
        ss << "Loaded bounding boxes: " << loadedBb_ << "/" <<  totalBb_ << std::endl;
        ss << "Loaded spatial meshes: " << loadedSMeshes_ << "/" << totalSMeshes_ << std::endl;
        feedback_.log = ss.str();
        server_.publishFeedback(feedback_);
        ss.str(std::string());
    }

    void addBoundingBoxes(const std::vector<BoundingBox>& bb_vec)
    {
        ROS_INFO("Adding bounding boxes to the planning scene...\n");

        for(size_t i = 0; i < totalBb_; i++){
            // Unpack the content of the ith element in the vector 
            BoundingBox next = bb_vec[i];
            // Initialize empty collision object
            moveit_msgs::CollisionObject co;

            // Populate collision object info
            co.header.frame_id = next.header.frame_id;
            co.id = next.name;

            // Define a box shape
            shape_msgs::SolidPrimitive box;
            box.type = shape_msgs::SolidPrimitive::BOX;

            // Specify box dims
            box.dimensions.resize(3);
            box.dimensions[0] = next.dims.x;
            box.dimensions[1] = next.dims.y;
            box.dimensions[2] = next.dims.z;

            // Add primitive and its pose to collision object
            co.primitives.push_back(box);
            co.primitive_poses.push_back(next.pose);

            // Add collision object to the scene
            ROS_INFO("Adding %s to the scene\n", next.name.c_str());
            co.operation = moveit_msgs::CollisionObject::ADD;
            current_scene_.addCollisionObjects({co});

            loadedBb_++;
            pushFeedback();
        }
    }

    void addSpatialMappingMeshes(const std::vector<SpatialMappingMesh>& ssmesh_vec)
    {
        ROS_INFO("Adding spatial meshes to the planning scene...\n");

        for(size_t i = 0; i < totalSMeshes_; i++){
            // Unpack the content of the ith element in the vector 
            SpatialMappingMesh next = ssmesh_vec[i];
            // Load mesh from binary stream with assimp with format int
            shapes::Mesh* m = 
                shapes::createMeshFromBinary((char*) next.mesh.data(), next.mesh.size(), "stl");
            
            if(m == nullptr){
                ROS_WARN("ManipObstaclesManager: the request contains an empty mesh or a mesh could not be loaded correctly\n");
                continue;
            }

            // Create shape message with the loaded mesh to pass it to move_group
            shape_msgs::Mesh mesh;
            shapes::ShapeMsg mesh_msg;
            shapes::constructMsgFromShape(m, mesh_msg);
            mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
            
            // Initialize new collision object
            // We load it once at a time to avoid overflowing the memory
            moveit_msgs::CollisionObject co;
            co.id = next.name;
            co.header.frame_id = next.header.frame_id;
            co.meshes.push_back(mesh);
            co.mesh_poses.push_back(next.pose);
            co.operation = co.ADD;
            current_scene_.addCollisionObjects({co});

            loadedSMeshes_++;
            pushFeedback();
        }
    }

    // Action server callback - register all the collision objects in the request into the current scene
    void registerCollisionObjs_CB(const RegisterCollisionObjectsGoalConstPtr &goal)
    {
        ROS_INFO("New goal received");
        ros::Rate r(1);
        Vector3d scale(1.0, 1.0, 1.0);

        loadedBb_ = 0;
        totalBb_ = goal->boundingBoxes.size();
        loadedSMeshes_ = 0;
        totalSMeshes_ = goal->spatialMappingMeshes.size();
        
        // Begin to log progress
        pushFeedback();

        // First add the bounding boxes
        addBoundingBoxes(goal->boundingBoxes);
        
        // Add the spatial mapping meshes to the planning scene the spatial mapping meshes 
        addSpatialMappingMeshes(goal->spatialMappingMeshes);

        // If all the obstacles were successfully loaded, set succeded
        // TODO: send a warning if some of the obstacles were not sent
        result_.log = "DONE - All the obstacles were successfully loaded";
        server_.setSucceeded(result_);
    }

protected:
    // Node handle
    ros::NodeHandle nh;
    // Action-server stuff
    actionlib::SimpleActionServer<RegisterCollisionObjectsAction> server_;
    std::string actionName_;
    RegisterCollisionObjectsFeedback feedback_;
    RegisterCollisionObjectsResult result_;
    // Counters for logging progress
    size_t loadedBb_{0};
    size_t totalBb_{0};
    size_t loadedSMeshes_{0};
    size_t totalSMeshes_{0};    
    // Planning interface to add the collision objects
    moveit::planning_interface::PlanningSceneInterface current_scene_;
};

int main(int argc, char** argv){
    // Initialize ros
    ros::init(argc, argv, "ManipObstaclesManager");

    // Create action server wrapper
    std::string coServerActionName = "RegisterCollisionObjects";
    ManipObstaclesManager collisionManager(coServerActionName);

    // Start spinning
    ros::spin();
    
    return 0;
}