//
// Created by kook on 22-12-30.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
        {
                {'w', {1, 0, 0, 0}},
                {'e', {1, 0, 0, -1}},
                {'a', {0, 0, 0, 1}},
                {'d', {0, 0, 0, -1}},
                {'q', {1, 0, 0, 1}},
                {'x', {-1, 0, 0, 0}},
                {'c', {-1, 0, 0, 1}},
                {'z', {-1, 0, 0, -1}},
                {'E', {1, -1, 0, 0}},
                {'W', {1, 0, 0, 0}},
                {'A', {0, 1, 0, 0}},
                {'D', {0, -1, 0, 0}},
                {'Q', {1, 1, 0, 0}},
                {'X', {-1, 0, 0, 0}},
                {'C', {-1, -1, 0, 0}},
                {'Z', {-1, 1, 0, 0}},
                {'t', {0, 0, 1, 0}},
                {'b', {0, 0, -1, 0}},
                {'s', {0, 0, 0, 0}},
                {'S', {0, 0, 0, 0}}
        };

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
        {
                {'4', {1.1, 1.1}},
                {'1', {0.9, 0.9}},
                {'5', {1.1, 1}},
                {'2', {0.9, 1}},
                {'6', {1, 1.1}},
                {'3', {1, 0.9}}
        };

// Reminder message
const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c
For Omni-directional movement of Mecanum wheels.
Just hold down the shift key:
---------------------------
   Q    W    E
   A    S    D
   Z    X    C
t : up (+z)
b : down (-z)
anything else : stop
4/1 : increase/decrease max speeds by 10%
5/2 : increase/decrease only linear speed by 10%
6/3 : increase/decrease only angular speed by 10%
CTRL-C to quit
)";

// Init variables
float speed(0.5); // Linear velocity (m/s)
float turn(1.0); // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "teleop_twist_keyboard");
    ros::NodeHandle nh;

    // Init cmd_vel publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Create Twist message
    geometry_msgs::Twist twist;

    printf("%s", msg);
    printf("\rCurrent: speed %f\tturn %f | Awaiting command...\r", speed, turn);

    while(true){

        // Get the pressed key
        key = getch();

        // If the key corresponds to a key in moveBindings
        if (moveBindings.count(key) == 1)
        {
            // Grab the direction data
            x = moveBindings[key][0];
            y = moveBindings[key][1];
            z = moveBindings[key][2];
            th = moveBindings[key][3];

            printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
        }

            // Otherwise if it corresponds to a key in speedBindings
        else if (speedBindings.count(key) == 1)
        {
            // Grab the speed data
            speed = speed * speedBindings[key][0];
            turn = turn * speedBindings[key][1];

            printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
        }

            // Otherwise, set the robot to stop
        else
        {
            x = 0;
            y = 0;
            z = 0;
            th = 0;

            // If ctrl-C (^C) was pressed, terminate the program
            if (key == '\x03')
            {
                printf("\n\n      Welcome to Guangdong University of Technology\n\n");
                break;
            }

            printf("\rCurrent: speed %f\tturn %f | Invalid command! %c", speed, turn, key);
        }

        // Update the Twist message
        twist.linear.x = x * speed;
        twist.linear.y = y * speed;
        twist.linear.z = z * speed;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th * turn;

        // Publish it and resolve any remaining callbacks
        pub.publish(twist);
        ros::spinOnce();
    }

    return 0;
}