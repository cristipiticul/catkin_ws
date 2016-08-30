#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

typedef std_srvs::EmptyRequest Request;
typedef std_srvs::EmptyResponse Response;

struct TransformSpecification {
    std::string target_frame;
    std::string source_frame;
};

#define NUM_TRANSFORMS 6
TransformSpecification transform_specifications[] = {
        { "/camera_rgb_optical_frame", "/bottle" },
        //{ "/camera_link", "/camera_rgb_optical_frame" },
        //{ "/wrist_roll", "/camera_link"},
        { "/base_footprint", "/camera_rgb_optical_frame" },
        { "/base_footprint", "/gripper_finger1" },
        { "/gripper_finger1", "/camera_rgb_optical_frame" },
        { "/base_footprint", "/virtual_endeffector"}, //this one is used in case we want to repeat the experiment
        { "/base_footprint", "/wrist_roll" }
};
FILE* save_files[NUM_TRANSFORMS];

const char save_dir[] = "/home/cristi/catkin_ws/src/transforms";
char current_save_dir[100];

bool save_transforms(Request &request, Response &response) {

    ROS_INFO("Received service call");
    for (int i = 0; i < NUM_TRANSFORMS; i++) {
        tf::TransformListener listener;
        tf::StampedTransform transform;
        try {
            ros::Time now = ros::Time(0);
            listener.waitForTransform(transform_specifications[i].target_frame,
                    transform_specifications[i].source_frame, now,
                    ros::Duration(10.0));
            listener.lookupTransform(transform_specifications[i].target_frame,
                    transform_specifications[i].source_frame, now, transform);
            fprintf(save_files[i], "%f %f %f\n", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            fprintf(save_files[i], "%f %f %f %f\n", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            exit(-1);
        }
    }
    ROS_INFO("Transforms saved successfully");

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "transforms_saver");
    ros::NodeHandle n;

    struct stat st = {0};
    if (stat(save_dir, &st) == -1) {
        mkdir(save_dir, 0777);
    }

    sprintf(current_save_dir, "%s/%u", save_dir, ros::Time::now().sec);
    if (stat(current_save_dir, &st) == -1) {
        mkdir(current_save_dir, 0777);
    } else {
        ROS_ERROR("The folder %s already exists!", current_save_dir);
        exit(-1);
    }

    for (int i = 0; i < NUM_TRANSFORMS; i++) {
        char filename[100];
        sprintf(filename, "%s/%d.txt", current_save_dir, i);
        save_files[i] = fopen(filename, "w");
        fprintf(save_files[i], "target: %s, source: %s:\n", transform_specifications[i].target_frame.c_str(), transform_specifications[i].source_frame.c_str());
    }

    ros::ServiceServer serviceClient = n.advertiseService("save_transform",
            save_transforms);

    ros::spin();

    for (int i = 0; i < NUM_TRANSFORMS; i++) {
        fclose(save_files[i]);
    }
    ros::shutdown();
    return 0;
}
