#include "object_placement.hpp"

void objectPlacing::sonarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    sonar_data.angle_min = msg->angle_min;
    sonar_data.angle_max = msg->angle_max;
    sonar_data.angle_increment = msg->angle_increment;
    sonar_data.ranges = msg->ranges;
}

void objectPlacing::ekfCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ekf_data.pose.pose.position.x = msg->pose.pose.position.x;
    ekf_data.pose.pose.position.y = msg->pose.pose.position.y;
    ekf_data.pose.pose.position.z = msg->pose.pose.position.z;
    ekf_data.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    ekf_data.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    ekf_data.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    ekf_data.pose.pose.orientation.w = msg->pose.pose.orientation.w;
}

void objectPlacing::darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {
    tuple<float,float> bearings;
    Vector2i map_coordinates;
    float depth;
    darknet_data.bounding_boxes = msg->bounding_boxes;
    vector<tuple<int,int,int,int,string>> list_of_bbox(darknet_data.bounding_boxes.size()); 
    for (int i = 0; i < darknet_data.bounding_boxes.size(); i++) {
        list_of_bbox[i] = tuple<int,int,int,int,string>(darknet_data.bounding_boxes[i].xmin,
        darknet_data.bounding_boxes[i].xmax,darknet_data.bounding_boxes[i].ymin,darknet_data.bounding_boxes[i].ymax,
        darknet_data.bounding_boxes[i].Class);      
    }
    for (int i = 0; i < list_of_bbox.size(); i++) {
        bearings = objectPlacing::calculate_bearings(list_of_bbox[i]);
        depth = objectPlacing::calculate_depth(bearings);
        map_coordinates = objectPlacing::calculate_map_coordinates(bearings,depth);
        if (get<4>(list_of_bbox[i]) == "bootlegger") {
            bootlegger.update_object(map_coordinates);
        }
        if (get<4>(list_of_bbox[i]) == "g-man") {
            gman.update_object(map_coordinates);
        }
        if (get<4>(list_of_bbox[i]) == "tommy-gun") {
            tommygun.update_object(map_coordinates);
        }
        if (get<4>(list_of_bbox[i]) == "badge") {
            badge.update_object(map_coordinates);
        }        
    }
}

void objectPlacing::publishCallback(const ros::TimerEvent& event) {
    // Filling msg
    objectPlacement_data.bootlegger_found = bootlegger.detected;
    objectPlacement_data.bootlegger_object_x = bootlegger.x_pos;
    objectPlacement_data.bootlegger_object_y = bootlegger.y_pos;
    
    objectPlacement_data.gman_found = gman.detected;
    objectPlacement_data.gman_object_x = gman.x_pos;
    objectPlacement_data.gman_object_y = gman.y_pos;

    objectPlacement_data.badge_found = badge.detected;
    objectPlacement_data.badge_object_x = badge.x_pos;
    objectPlacement_data.badge_object_y = badge.y_pos;

    objectPlacement_data.tommygun_found = tommygun.detected;
    objectPlacement_data.tommygun_object_x = tommygun.x_pos;
    objectPlacement_data.tommygun_object_y = tommygun.y_pos;

    // Publishing
    objectPlacement_pub.publish(objectPlacement_data);
}


tuple<float,float> objectPlacing::calculate_bearings(tuple<int,int,int,int,string> bbox) {
    float max_bearing, min_bearing;
    max_bearing = objectPlacing::pixel_to_sonar(get<0>(bbox));
    min_bearing = objectPlacing::pixel_to_sonar(get<1>(bbox));
    return tuple<float,float>(min_bearing,max_bearing);

}
float objectPlacing::pixel_to_sonar(int pixel_width) {
    if (d != -100.0) {
        return pow((float)pixel_width,3)*a + pow((float)pixel_width,2)*b + pixel_width*c + d;
    }    
    if (c != -100.0) {
        return pow((float)pixel_width,2)*a + pixel_width*b + c;
    }
    return pixel_width*a + b;
}

float objectPlacing::calculate_depth(tuple<float,float> bearings) {
    list<float> depth;
    float closest_depth = INFINITY;
    float current_angle = sonar_data.angle_min;
    for (int i = 0; i < sonar_data.ranges.size(); i++) {
        if ((current_angle > get<0>(bearings)) && (current_angle > get<1>(bearings))) {
            depth.push_back(sonar_data.ranges[i]);
            if (sonar_data.ranges[i] < closest_depth) {
                closest_depth = sonar_data.ranges[i];
            }
        }
        current_angle += sonar_data.angle_increment;
    }
    float count = 0.0;
    float sum =  0.0;
    for (auto itr = depth.begin(); itr != depth.end(); itr++) {
        if ( *itr < (closest_depth+0.3)) {
            sum += *itr;
            count += 1;
        }
    }
    return sum/count;
}

Vector2i objectPlacing::calculate_map_coordinates(tuple<float,float> bearings, float depth) {
    float bearing, tmp_width, tmp_height;
    Vector2i manta_position,rotated_point;
    Vector3f manta_angles;
    Vector2f point, center; 
    vector<int> x,y;
    // Calculating current positions and euler angles of manta
    manta_position[0] = (int)(500.0/2.0 - ekf_data.pose.pose.position.x*5);
    manta_position[1] = (int)(500.0/2.0 - ekf_data.pose.pose.position.y*5);
    manta_angles = quaternion_to_euler(ekf_data.pose.pose.orientation.x,ekf_data.pose.pose.orientation.y,
        ekf_data.pose.pose.orientation.z,ekf_data.pose.pose.orientation.w);
    
    // Projecting point from the sonar onto a 2D-plane given range and angle of current ping
    bearing = (get<0>(bearings)+get<1>(bearings))/2.0;
    tmp_width = sin(bearing)*depth;
    tmp_height = sqrt(pow(depth,2) - pow(tmp_width,2));
    tmp_width = manta_position[1] - (tmp_width*5);
    tmp_height = manta_position[0] - (tmp_height*5);

    // Transforming point between local frame and manta/odom
    point[0] = tmp_height;
    point[1] = tmp_width;
    center[0] = (float)manta_position[0];
    center[1] = (float)manta_position[1];

    return rotatePointAroundCenter(point, center, manta_angles[2]);
}




void objectClass::update_object(Vector2i map_coordinates) {
        if (detected == true) {
            if ((map_coordinates[0] < x_pos+3) && (map_coordinates[1] > x_pos - 3)) {
                x_pos = x_pos*0.8 + map_coordinates[0]*0.2;
                y_pos = y_pos*0.8 + map_coordinates[1]*0.2;
            }
        }
        else {
            queue += 1;
            if (queue >= 5) {
                detected = true;
            }
            x_pos += 0.2*map_coordinates[0];
            y_pos += 0.2*map_coordinates[1];
        }
    }  









































int main(int argc, char** argv) {
    ros::init(argc, argv, "object_placement");
    objectPlacing ic(argc, argv);
    ros::spin();
    return 0;
}