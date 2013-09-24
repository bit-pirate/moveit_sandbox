#!/usr/bin/env python
import argparse
import rospy
import actionlib
import moveit_msgs.msg as moveit_msgs
import object_recognition_msgs.srv as object_recognition_srvs
import geometry_msgs
import shape_msgs


COLLISION_OBJECT_TOPIC = "/collision_object"
#COLLISION_OBJECT_TOPIC = "/korus/collision_object"
OBJECT_INFORMATION_TOPIC = "/get_object_info"
#OBJECT_INFORMATION_TOPIC = "/korus/get_object_info"

def main():
    rospy.init_node('add_collison_objects')
    
    parser = argparse.ArgumentParser(description='Adds an object from household objects database to the MoveIt planning scene')
    parser.add_argument('pos_x', type = float, help='X value of the object position')
    parser.add_argument('pos_y', type = float, help='Y value of the object position')
    parser.add_argument('pos_z', type = float, help='Z value of the object position')
    parser.add_argument('--remove', dest='remove', action='store_true', default=False, help='Remove all objects from the scene')
    args=parser.parse_args()
    
    pub_collision_object = rospy.Publisher(COLLISION_OBJECT_TOPIC,
                                           moveit_msgs.CollisionObject,
                                           latch = True)
    while (pub_collision_object.get_num_connections() < 1) and not rospy.is_shutdown():
        rospy.loginfo("Waiting for planning scene listening to '" + str(COLLISION_OBJECT_TOPIC) + "' ...")
        rospy.sleep(0.5)
    
    collision_object = moveit_msgs.CollisionObject()
    collision_object.header.stamp = rospy.Time.now()
    collision_object.header.frame_id = "/base_footprint";
    
    if args.remove:
        collision_object.operation = moveit_msgs.CollisionObject.REMOVE
        rospy.loginfo('Removing object ...')
    else:
        rospy.loginfo("Waiting for '" + str(OBJECT_INFORMATION_TOPIC) + "' service ... ")
        rospy.wait_for_service(OBJECT_INFORMATION_TOPIC)
        rospy.loginfo("'" + str(OBJECT_INFORMATION_TOPIC) + "' service available.")
        info_response = object_recognition_srvs.GetObjectInformationResponse()
        srv_client = rospy.ServiceProxy(OBJECT_INFORMATION_TOPIC, object_recognition_srvs.GetObjectInformation())
        try:
            info_request = object_recognition_srvs.GetObjectInformationRequest()
            info_request.type.key = "18807" # Coke can
            info_request.type.db = '{"host":"localhost","module":"object_recognition_tabletop","name":"household_objects-0.6","password":"yujin","port":"5432","type":"ObjectDbSqlHousehold","user":"yujin"'
            info_response = srv_client.call(info_request)
        except rospy.ServiceException, e:
            rospy.logerr("Service did not process request: " + str(e))
            return
        rospy.loginfo("Object information retrieved.")
        
        ''' prepare collision object message'''
        collision_object.id = info_response.information.name
        collision_object.type = info_request.type
        ''' add a solid primitive shape '''
        object_shape = shape_msgs.msg.SolidPrimitive()
        object_shape.type = shape_msgs.msg.SolidPrimitive.CYLINDER
        object_shape.dimensions.append(0.1) # CYLINDER_HEIGHT
        object_shape.dimensions.append(0.05) # CYLINDER_RADIUS
        collision_object.primitives.append(object_shape)
        shape_pose = geometry_msgs.msg.Pose()
        shape_pose.position.x = args.pos_x
        shape_pose.position.y = args.pos_y
        shape_pose.position.z = args.pos_z + object_shape.dimensions[0] / 2 # reference is at the centre of the mesh
        shape_pose.orientation.w = 1.0
        collision_object.primitive_poses.append(shape_pose)
        ''' add a mesh from the household objects database '''
        collision_object.meshes.append(info_response.information.ground_truth_mesh)
        mesh_pose = geometry_msgs.msg.Pose()
        mesh_pose.position.x = args.pos_x
        mesh_pose.position.y = args.pos_y
        mesh_pose.position.z = args.pos_z # reference is at the bottom of the mesh
        mesh_pose.orientation.w = 1.0
        collision_object.mesh_poses.append(mesh_pose)
        
        collision_object.operation = moveit_msgs.CollisionObject.ADD
        rospy.loginfo('Adding object ...')
    pub_collision_object.publish(collision_object)
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass