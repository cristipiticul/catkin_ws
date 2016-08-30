#include <math.h>
#include <stdio.h>
#include "ros/ros.h"
#include "tf/transform_datatypes.h"


// Rotation:
// q1: -90 degrees around Ox
// q2: 90 degrees around (the new?) Oy
int main() {
/*    tf::Quaternion q = tf::createIdentityQuaternion();
    float sin45 = sqrt(2.0f) / 2.0f;
    float cos45 = sqrt(2.0f) / 2.0f;
    tf::Quaternion q1(-sin45, 0.0f, 0.0f, cos45);
    tf::Quaternion q2(0.0f, sin45, 0.0f, cos45);
    q = q1 * q2;
    printf("%.3f %.3f %.3f %.3f", q.x(), q.y(), q.z(), q.w()); */
    printf("Commands:\n");
    printf(" r : resets the current quaternion to identity.\n");
    printf(" ? : prints the current quaternion.\n");
    printf(" * : multiplies the current quaternion by another quaternion.\n");
    printf(" c : prints the quaternion created from RPY rotation. (It does not modify the current quaternion)\n");
    printf(" q : quit\n");
    printf(" i : inverse\n");
    tf::Quaternion q = tf::createIdentityQuaternion();
    char command;
    do {
        printf("Awaiting your command, master: ");
        scanf(" %c", &command);
        switch (command) {
        case 'r':
            printf("Resetting the quaternion.\n");
            q = tf::createIdentityQuaternion();
            break;
        case '?':
            printf("The quaternion is: x: %.4f y: %.4f z: %.4f w: %.4f.\n", q.getX(), q.getY(), q.getZ(), q.getW());
            printf("rosrun tf static_transform_publisher 0 -0.1 0 %.4f %.4f %.4f %.4f base_footprint camera 100\n", q.getX(), q.getY(), q.getZ(), q.getW());
            break;
        case '*':
            {
                printf("Multiply by x y z w:\n");
                double x, y, z, w;
                scanf("%lf %lf %lf %lf", &x, &y, &z, &w);
                tf::Quaternion q2(x, y, z, w);
                q = q * q2;
                printf("The new quaternion is: x: %.4f y: %.4f z: %.4f w: %.4f.\n", q.getX(), q.getY(), q.getZ(), q.getW());
            }
            break;
        case 'c':
            {
                printf("Quaternion from R P Y:\n");
                double r, p, y;
                scanf("%lf %lf %lf", &r, &p, &y);
                tf::Quaternion q2 = tf::createQuaternionFromRPY(r, p, y);
                printf("The new quaternion is: x: %.4f y: %.4f z: %.4f w: %.4f.\n", q2.getX(), q2.getY(), q2.getZ(), q2.getW());
            }
            break;
        case 'i':
        {
            printf("Inversing quaternion:\n");
            q = q.inverse();
            break;
        }
        default:
            printf("Unknown command recieved: \"%c\".\n", command);
        }
    } while (command != 'q');
    return 0;
}
