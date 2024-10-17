#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

using namespace std;
ros::Publisher pub;

double euclideanDistance(vector<int> first_point, vector<int> second_point) {
    double distance = sqrt(pow(first_point[1] - second_point[1], 2) + (first_point[0] - second_point[0], 2));
    return distance;
}

int getIndex(vector<int> coordinates, vector<int> maze_dims) {
    int index = coordinates[0]*maze_dims[1] + coordinates[1];
    return index;
}

vector<int> getCoordinates(int index, vector<int> maze_dims) {
    int x = index/maze_dims[1];
    int y = index%maze_dims[1];
    return {x, y};
}

vector<int> getAdjacentCells(int index, vector<int> maze_dims) {
    vector<vector<int>> movements = {
                                    {-1, -1},{-1, 0},{-1, 1},
                                    {0, -1},{0, 1},
                                    {1, -1},{1, 0},{1, 1}};

    int m = maze_dims[0];
    int n = maze_dims[1];

    int row = index / n;
    int col = index % n;

    vector<int> adjacent_cells;

    for (vector<int> movement : movements) {
        int new_row = row + movement[0];
        int new_col = col + movement[1];

        if (new_row >= 0 && new_row < m && new_col >= 0 && new_col < n) {
            int adjacent_cell = new_row * n + new_col; 
            adjacent_cells.push_back(adjacent_cell);
        }
    }

    return adjacent_cells;
}

vector<int> pathFinder(vector<int> start_point, vector<int> end_point, const vector<int> &maze, vector<int> maze_dims) {
    vector<vector<int>> movements = {
                                    {-1, -1},{-1, 0},{-1, 1},
                                    {0, -1},{0, 1},
                                    {1, -1},{1, 0},{1, 1}};

    int m = maze_dims[0];
    int n = maze_dims[1];

    vector<pair<double, int>> priority_que;
    vector<bool> visited(m*n, false);
    vector<int> parent(m*n, -1);
    vector<double> G_costs(m*n, INT32_MAX);
    vector<double> F_costs(m*n, INT32_MAX);

    int start_index = getIndex(start_point, maze_dims);
    int end_index = getIndex(end_point, maze_dims);

    priority_que.push_back({0, start_index});
    G_costs[start_index] = 0;
    F_costs[start_index] = euclideanDistance(start_point, end_point);

    while(!priority_que.empty()) {
        double min_H_cost = INT32_MAX;
        int min_H_cost_index = -1;

        //lowest H_cost
        for (int i = 0; i < priority_que.size(); i++) {
            if( priority_que[i].first < min_H_cost) {
                min_H_cost = priority_que[i].first;
                min_H_cost_index = i;
            }
        }

        int current_index = priority_que[min_H_cost_index].second;
        priority_que.erase(priority_que.begin() + min_H_cost_index);

        //end reached?
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        int count = 0;
        if(current_index == end_index) {
            vector<int> path;
            while (current_index != -1) {
                path.push_back(current_index);
                current_index = parent[current_index];
            }
            
            reverse(path.begin(), path.end());

            //packing and publishing the data
            for (int i = 0; i < path.size(); i++) {
                vector<int> coord = getCoordinates(path[i],maze_dims);
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.stamp = ros::Time::now();
                pose_stamped.header.frame_id = "map";
                pose_stamped.pose.position.x = coord[1];
                pose_stamped.pose.position.y = coord[0];
                pose_stamped.pose.position.z = 0;
                pose_stamped.pose.orientation.w = 1.0;
                path_msg.poses.push_back(pose_stamped);
            }
            cout << "Path found" << endl;
            cout << "Steps taken: " << path.size() << endl;
            pub.publish(path_msg);

            return path;
        }

        //end isnt reached
        vector<int> current_point = getCoordinates(current_index, maze_dims);
        visited[current_index] = true;

        //get valid children nodes and add to priority
        vector<int> adjacent_indexes = getAdjacentCells(current_index, maze_dims);
        for(int new_index : adjacent_indexes) {
            if(maze[new_index] != 1 && !visited[new_index]) {
                int new_G_cost = G_costs[current_index] + 1;
                double new_F_cost = euclideanDistance(getCoordinates(new_index, maze_dims), end_point);
                double new_H_cost = new_G_cost + new_F_cost; 

                //path optimization: only add to priority when the cost is better
                if (new_H_cost < G_costs[new_index] + F_costs[new_index]) {
                    G_costs[new_index] = new_G_cost;
                    F_costs[new_index] = new_F_cost;
                    parent[new_index] = current_index;
                    priority_que.push_back({new_H_cost, new_index});
                    
                }   
            }
        }
    }

    return vector<int>();
}

// int main() {
//     const vector<int> maze = {
//                 0, 0, 0, 0, 0, 0, 1, 0, 0,
//                 0, 0, 0, 0, 0, 1, 1, 0, 0,
//                 0, 0, 1, 1, 0, 1, 0, 0, 0,
//                 0, 1, 1, 0, 0, 0, 0, 1, 0,
//                 0, 0, 0, 0, 0, 0, 0, 1, 0
//                 };

//                 //          0  0  0  1  0  0
//                 // 0  0  0     0  1  1  0  0 
//                 // 0  0  1  1     1        0
//                 // 0  1  1  0  0     0  1  
//                 // 0  0  0  0  0  0  0  1  
    
//     vector<int> path = pathFinder({0,0}, {4,8}, maze, {5,9});

//     for (int i = 0; i < path.size(); i++)
//     {
//         cout << path[i] << " ";
//     }
    

//     return 0;
// }
void callback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    cout << "Map initialized" << endl;
    vector<int> map2;
    int width = map->info.width;
    int height = map->info.height;
    for(int i=0;i<width*height;i++) {
        if (map->data[i] == 100) {
            map2.push_back(1);
        }
        else {
            map2.push_back(0);
        }
    }
    vector<int> path = pathFinder({0,0}, {height-1,width-1}, map2, {height,width});
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "path_finder");

    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("map",1000,callback);
    pub = nh.advertise<nav_msgs::Path>("path",100);

    ros::spin();

    return 0;
}

