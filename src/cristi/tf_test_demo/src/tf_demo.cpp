#include <tf/tf.h>
#include <stdio.h>
#include <vector>

using namespace std;
using namespace tf;

const int number_of_measurements = 5;
const int number_of_transforms = 4;
const char transforms_directory[] =
        "/home/cristi/catkin_ws/src/transforms/1462266925";
const double THRESHOLD = 0.03;

vector<Transform> readTransforms(const char *filename) {
    vector<Transform> result;
    FILE *f = fopen(filename, "r");
    char first_line[100];
    fscanf(f, "%[^\n]", first_line);
    printf("reading %s\n", first_line);

    for (int i = 0; i < number_of_measurements; i++) {
        double x, y, z;
        fscanf(f, "%lf", &x);
        fscanf(f, "%lf", &y);
        fscanf(f, "%lf", &z);
        Vector3 translation(x, y, z);

        double rx, ry, rz, rw;
        fscanf(f, "%lf", &rx);
        fscanf(f, "%lf", &ry);
        fscanf(f, "%lf", &rz);
        fscanf(f, "%lf", &rw);
        Quaternion rotation(rx, ry, rz, rw);

        Transform transform(rotation, translation);
        result.push_back(transform);
    }

    fclose(f);
    return result;
}

vector<vector<Transform> > readAllTransforms() {
    vector<vector<Transform> > result;
    for (int i = 0; i < number_of_transforms; i++) {
        char filename[100];
        sprintf(filename, "%s/%d.txt", transforms_directory, i);
        vector<Transform> transforms = readTransforms(filename);
        if (transforms.size() != number_of_measurements) {
            ROS_ERROR(
                    "Something went wrong while reading the measurements. Expected to read %d measurements, got %d transforms\n",
                    number_of_measurements, (int)transforms.size());
            exit(-1);
        }
        result.push_back(transforms);
    }
    return result;
}

void printTransform(Transform t) {
    printf("%f %f %f\n", t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z());
    printf("%f %f %f %f\n", t.getRotation().x(), t.getRotation().y(), t.getRotation().z(), t.getRotation().w());
}

inline double findTransformError(vector<Transform> camera_transforms, vector<Transform> bottle_transforms, Transform delta) {
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;
    vector<double> xs;
    vector<double> ys;
    vector<double> zs;
    for (int i = 0; i < number_of_measurements; i++) {
        Transform t1;
        t1.mult(camera_transforms[i], delta);
        Transform t2;
        t2.mult(t1, bottle_transforms[i]);
        xs.push_back(t2.getOrigin().x());
        ys.push_back(t2.getOrigin().y());
        zs.push_back(t2.getOrigin().z());
        //printf("zs[i]:%f\n", zs[i]);
    }
    for (int i = 0; i < number_of_measurements; i++) {
        sum_x += xs[i];
        sum_y += ys[i];
        sum_z += zs[i];
    }
    double average_x = sum_x / number_of_measurements;
    double average_y = sum_y / number_of_measurements;
    double average_z = sum_z / number_of_measurements;
    double error = 0;
    for (int i = 0; i < number_of_measurements; i++) {
        double ex = (xs[i] - average_x) * (xs[i] - average_x);
        double ey = (ys[i] - average_y) * (ys[i] - average_y);
        double ez = (zs[i] - average_z) * (zs[i] - average_z);
        if (ex >= THRESHOLD) {
            error += 0.1;
        }
        if (ey >= THRESHOLD) {
            error += 0.1;
        }
        if (ez >= THRESHOLD) {
            error += 0.1;
        }
        error += ex + ey + ez;
    }
    return error;
}

double errors[12][12][12][12][12][12];
int main() {
    vector<vector<Transform> > transforms = readAllTransforms();

    //!! Important: THE EXPERIMENT SHOULD BE DONE WITH THE SHERE!! even though the frame is named "bottle"

    //we need base->camera_rgb_optical_frame and camera_rgb_optical_frame->bottle for each measurement.
    //We will add an intermediary transform called "Delta" between camera_rgb_optical_frame.
    //This will translate and rotate with small amounts.
    //The transform which performs the best will be found.

    //The transform performance is measured by the error in the position of the bottle:
    //The bottle is standing, so base_link -> bottle should be constant.
    //Compute the base_link -> bottle transform for each measurement.
    //Find the average bottle position.
    //Compute the error as the squared sum of all the errors (on x, y, z axis)
    //Minimum error => the best transform

    //1. Computing base->camera_rgb_optical_frame transforms
    vector<Transform> camera_transforms;
    for (int i = 0; i < number_of_measurements; i++) {
        camera_transforms.push_back(transforms[1][i]);
    }
    /* Old version: it was computed from base->wrist_roll->camera
    vector<Transform> camera_transforms;
    for (int i = 0; i < number_of_measurements; i++) {
        Transform t1;
        // base->wrist_roll->camera_link
        t1.mult(transforms[3][i], transforms[2][i]);
        // base->camera_rgb_optical_frame
        Transform t2;
        t2.mult(t1, transforms[1][i]);
        camera_transforms.push_back(t2);
    }

    printf("Camera transforms:\n");
    for (int i = 0; i < number_of_measurements; i++) {
        printTransform(camera_transforms[i]);
        printf("\n");
    }*/

    //2. Saving camera_rgb_optical_frame->bottle transforms
    vector<Transform> bottle_transforms;
    for (int i = 0; i < number_of_measurements; i++) {
        bottle_transforms.push_back(transforms[0][i]);
    }

    //3. Here comes the fun. Try with each delta:
    const int steps = 5;
    const double delta_coordinate = 0.003;
    const double delta_angle = 0.02; // ~ 1 degree
    const double center_x = 0.0;
    const double center_y = 0.0;
    const double center_z = 0.0;
    const double center_rx = 0.0;
    const double center_ry = 0.0;
    const double center_rz = 0.0;
    double smallestError = 10.0;
    Transform bestDelta;
    for (int dxi = -steps; dxi <= steps; dxi++) {
        for (int dyi = -steps; dyi <= steps; dyi++) {
            for (int dzi = -steps; dzi <= steps; dzi++) {
                for (int drxi = -steps; drxi <= steps; drxi++) {
                    for (int dryi = -steps; dryi <= steps; dryi++) {
                        for (int drzi = -steps; drzi <= steps; drzi++) {
                            double dx = center_x + dxi * delta_coordinate;
                            double dy = center_y + dyi * delta_coordinate;
                            double dz = center_z + dzi * delta_coordinate;

                            double drx = center_rx + drxi * delta_angle;
                            double dry = center_ry + dryi * delta_angle;
                            double drz = center_rz + drzi * delta_angle;

                            Transform delta(tf::createQuaternionFromRPY(drx, dry, drz),
                                    tf::Vector3(dx, dy, dz));

                            double error = findTransformError(camera_transforms, bottle_transforms, delta);
                            errors[dxi + steps][dyi + steps][dzi + steps][drxi + steps][dryi + steps][drzi + steps] = error;
                        }
                    }
                }
            }
        }
    }

    // For each element in "errors", add half of its neighbours' errors
    int mat_size = 2 * steps + 1;
    for (int i1 = 0; i1 < mat_size; i1++) {
    for (int i2 = 0; i2 < mat_size; i2++) {
    for (int i3 = 0; i3 < mat_size; i3++) {
    for (int i4 = 0; i4 < mat_size; i4++) {
    for (int i5 = 0; i5 < mat_size; i5++) {
    for (int i6 = 0; i6 < mat_size; i6++) {
        int count_neighbours = 2;
        double neighbour_errors = 0;
        if (i1 > 0) {
            count_neighbours++;
            neighbour_errors += errors[i1 - 1][i2][i3][i4][i5][i6];
        }
        if (i1 < mat_size - 1) {
            count_neighbours++;
            neighbour_errors += errors[i1 + 1][i2][i3][i4][i5][i6];
        }

        if (i2 > 0) {
            count_neighbours++;
            neighbour_errors += errors[i1][i2 - 1][i3][i4][i5][i6];
        }
        if (i2 < mat_size - 1) {
            count_neighbours++;
            neighbour_errors += errors[i1][i2 + 1][i3][i4][i5][i6];
        }

        if (i3 > 0) {
            count_neighbours++;
            neighbour_errors += errors[i1][i2][i3 - 1][i4][i5][i6];
        }
        if (i3 < mat_size - 1) {
            count_neighbours++;
            neighbour_errors += errors[i1][i2][i3 + 1][i4][i5][i6];
        }

        if (i4 > 0) {
            count_neighbours++;
            neighbour_errors += errors[i1][i2][i3][i4 - 1][i5][i6];
        }
        if (i4 < mat_size - 1) {
            count_neighbours++;
            neighbour_errors += errors[i1][i2][i3][i4 + 1][i5][i6];
        }

        if (i5 > 0) {
            count_neighbours++;
            neighbour_errors += errors[i1][i2][i3][i4][i5 - 1][i6];
        }
        if (i5 < mat_size - 1) {
            count_neighbours++;
            neighbour_errors += errors[i1][i2][i3][i4][i5 + 1][i6];
        }

        if (i6 > 0) {
            count_neighbours++;
            neighbour_errors += errors[i1][i2][i3][i4][i5][i6 - 1];
        }
        if (i6 < mat_size - 1) {
            count_neighbours++;
            neighbour_errors += errors[i1][i2][i3][i4][i5][i6 + 1];
        }
        double error = errors[i1][i2][i3][i4][i5][i6] + neighbour_errors / count_neighbours / 2;
        if (error < smallestError) {
            double dx = center_x + (i1 - steps) * delta_coordinate;
            double dy = center_y + (i2 - steps) * delta_coordinate;
            double dz = center_z + (i3 - steps) * delta_coordinate;

            double drx = center_rx + (i4 - steps) * delta_angle;
            double dry = center_ry + (i5 - steps) * delta_angle;
            double drz = center_rz + (i6 - steps) * delta_angle;

            Transform delta(tf::createQuaternionFromRPY(drx, dry, drz),
                    tf::Vector3(dx, dy, dz));
            smallestError = error;
            bestDelta = delta;
        }
    }
    }
    }
    }
    }
    }

    printf("Smallest error: %f\n", smallestError);
    printf("Best delta:\n");
    printTransform(bestDelta);
    printf("\n");

    // 4. Find new gripper_finger1 -> camera_rgb_optical_frame
    // assume gripper_finger1->camera_rgb_optical_frame is constant.
    Transform gripper_finger1_to_optical_frame;
    gripper_finger1_to_optical_frame = transforms[3][0];
    Transform new_gripper_finger1_to_optical_frame;
    new_gripper_finger1_to_optical_frame.mult(gripper_finger1_to_optical_frame, bestDelta);

    printf("New gripper finger1 to optical frame:\n");
    printTransform(new_gripper_finger1_to_optical_frame);
    printf("\n");

    /**
    // a->b
    Transform transform;

    transform.setOrigin(Vector3(1.0, 2.0, 3.0));
    transform.setRotation(tf::createQuaternionFromRPY(3.1415 / 2.0, 0, 0));

    // b->c
    Transform transform2;

    transform2.setOrigin(Vector3(5.0, 6.0, 7.0));
    transform2.setRotation(createIdentityQuaternion());

    //a->c
    Transform transform3;

    transform3.mult(transform, transform2);

    //p from a->c
    Point p(2.0, 2.0, 2.0);
    Point p_transformed = transform3(p);
    printf("%f %f %f\n", p.x(), p.y(), p.z());
    printf("%f %f %f\n", p_transformed.x(), p_transformed.y(),
            p_transformed.z());

    Vector3 origin = transform3.getOrigin();
    Quaternion rotation = transform3.getRotation();

    printf("%f %f %f\n", origin.x(), origin.y(), origin.z());
    printf("%f %f %f %f\n", rotation.x(), rotation.y(), rotation.z(),
            rotation.w());
    */
    return 0;
}
