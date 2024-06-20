#include "tpg.h"
#include "adg.h"
#include "instance.h"

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include <vector>

#include "hardware_interface.h"

class MotoPlusExecutor
{
public:
    MotoPlusExecutor() {
        joint_pos_cmd_a = new double[6]();
        joint_pos_cmd_b = new double[6]();
        joint_pos_fbk_a = new double[6]();
        joint_pos_fbk_b = new double[6]();
    }

    ~MotoPlusExecutor() {
        delete[] joint_pos_cmd_a;
        delete[] joint_pos_cmd_b;
        delete[] joint_pos_fbk_a;
        delete[] joint_pos_fbk_b;
    }

    bool init(const std::string &tpg_path, const std::string &type) {
        std::ifstream ifs(tpg_path);
        if (!ifs.is_open()) {
            ROS_ERROR("Failed to open file: %s", tpg_path.c_str());
            return false;
        }
        if (type=="adg") {
            auto adg = std::make_shared<TPG::ADG>();
            boost::archive::text_iarchive ia(ifs);
            ia >> adg;
            tpg_ = adg;
        } else if (type=="tpg") {
            tpg_ = std::make_shared<TPG::TPG>();
            boost::archive::text_iarchive ia(ifs);
            ia >> tpg_;
        } else {
            tpg_ = std::make_shared<TPG::TPG>();
            boost::archive::text_iarchive ia(ifs);
            ia >> *tpg_;
        }


        ROS_INFO("Loaded TPG from %s with %d %d nodes", tpg_path.c_str(), tpg_->getNumNodes(0), tpg_->getNumNodes(1));

        double flowtime, makespan;
        traj_.joint_names.resize(14);
        tpg_->setSyncJointTrajectory(traj_, flowtime, makespan);
        
        ROS_INFO("Initialized TPG with flowtime: %f, makespan: %f", flowtime, makespan);
        return true;
    }

    void print_joint_pos()
    {
        printf("Robot A joint feedback: %f %f %f %f %f %f\n", joint_pos_fbk_a[0], joint_pos_fbk_a[1], joint_pos_fbk_a[2], joint_pos_fbk_a[3], joint_pos_fbk_a[4], joint_pos_fbk_a[5]);
        printf("Robot B joint feedback: %f %f %f %f %f %f\n", joint_pos_fbk_b[0], joint_pos_fbk_b[1], joint_pos_fbk_b[2], joint_pos_fbk_b[3], joint_pos_fbk_b[4], joint_pos_fbk_b[5]);
    }

    void print_target_pos()
    {
        printf("Robot A joint targets: %f %f %f %f %f %f\n", joint_pos_cmd_a[0], joint_pos_cmd_a[1], joint_pos_cmd_a[2], joint_pos_cmd_a[3], joint_pos_cmd_a[4], joint_pos_cmd_a[5]);
        printf("Robot B joint targets: %f %f %f %f %f %f\n", joint_pos_cmd_b[0], joint_pos_cmd_b[1], joint_pos_cmd_b[2], joint_pos_cmd_b[3], joint_pos_cmd_b[4], joint_pos_cmd_b[5]);
    }
    
    void send_pos_cmd() {
        robot_hw_.setJointPosCmd(joint_pos_cmd_a, robot_index_b);
        robot_hw_.setJointPosCmd(joint_pos_cmd_b, robot_index_a);
    }

    void goto_start() {
        std::vector<double> pos0 = traj_.points[0].positions;
        double t = 0;

        while (true) {
            robot_hw_.getJointPos(joint_pos_fbk_a, robot_index_b);
            robot_hw_.getJointPos(joint_pos_fbk_b, robot_index_a);
            printf("t = %f\n", t);
            print_joint_pos();

            // check if the robot is at the start pos
            double delta_a = 0, delta_b = 0;
            for (int i = 0; i < 6; i++) {
                delta_a += (pos0[i] - joint_pos_fbk_a[i]) * (pos0[i] - joint_pos_fbk_a[i]);
                delta_b += (pos0[i+7] - joint_pos_fbk_b[i]) * (pos0[i+7] - joint_pos_fbk_b[i]);
            }
            delta_a = sqrt(delta_a);
            delta_b = sqrt(delta_b);
            if (delta_a < 0.01 && delta_b < 0.01) {
                break;
            }

            //move to the start pos at 1m/s
            for (int i = 0; i < 6; i++) {
                joint_pos_cmd_a[i] = joint_pos_fbk_a[i] + (pos0[i] - joint_pos_fbk_a[i]) / delta_a * std::min(delta_a, 0.004 / scale_);
                joint_pos_cmd_b[i] = joint_pos_fbk_b[i] + (pos0[i+7] - joint_pos_fbk_b[i]) / delta_b * std::min(delta_b, 0.004 / scale_);
            }

            //send_pos_cmd();

            t += 0.004;
            usleep(4000);
        }

        return;
    }

    void set_scale(double scale) {
        scale_ = scale;
    }


    void sync_exec() {
        double dt = tpg_->getConfig().dt;
        
        int idx = 0;
        auto & points = traj_.points;
        
        if (points.size() < 2) {
            return;
        }

        std::vector<double> pos0 = points[idx].positions;
        std::vector<double> pos = pos0;
        std::vector<double> vel(14, 0);
        double t = 0;
        
        while (true) {
            if (t > points[idx+1].time_from_start.toSec() * scale_) {
                idx++;
                if ((idx+1) == points.size()) {
                    break;
                }
                pos0 = points[idx].positions;
            }

            // interpolate between two points
            for (int i = 0; i < 6; i++) {
                double tgoal = points[idx+1].time_from_start.toSec() * scale_;
                double tstart = points[idx].time_from_start.toSec() * scale_;
                double alpha = (t - tstart) / (tgoal-tstart);
                pos[i] = pos0[i] + (points[idx+1].positions[i] - pos0[i]) * alpha;
                pos[7+i] = pos0[7+i] + (points[idx+1].positions[7+i] - pos0[7+i]) * alpha;
            }   

            for (int i = 0; i < 6; i++) {
                vel[i] = pos[i] - joint_pos_cmd_a[i];
                vel[i+7] = pos[i+7] - joint_pos_cmd_b[i];
            }
            //printf("Vel A: %.4f %.4f %.4f %.4f %.4f %.4f\n", vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]);
            //printf("Vel B: %.4f %.4f %.4f %.4f %.4f %.4f\n", vel[7], vel[8], vel[9], vel[10], vel[11], vel[12]);

            std::copy(pos.begin(), pos.begin()+6, joint_pos_cmd_a);
            std::copy(pos.begin()+7, pos.begin()+13, joint_pos_cmd_b);

            send_pos_cmd();

            // print the set joint positions with printf
            printf("t = %f\n", t);
            print_target_pos();

            t += 0.004;
            usleep(4000);
        }

    }

    void tpg_exec() {
        double dt = tpg_->getConfig().dt;

        // currently reached node index
        int ida = 0;
        int idb = 0;

        // current targets
        std::shared_ptr<TPG::Node> nodea = tpg_->getStartNode(0);
        std::shared_ptr<TPG::Node> nodeb = tpg_->getStartNode(1);

        std::vector<double> joint_goal_a = nodea->pose.joint_values;
        std::vector<double> joint_goal_b = nodeb->pose.joint_values;
        
        double dta = 0;
        double dtb = 0;

        while (true) {

            robot_hw_.getJointPos(joint_pos_fbk_a, robot_index_b);
            robot_hw_.getJointPos(joint_pos_fbk_b, robot_index_a);
            double dist_a = 0, dist_b = 0;
            for (int i = 0; i < 6; i++) {
                dist_a += (joint_goal_a[i] - joint_pos_fbk_a[i]) * (joint_goal_a[i] - joint_pos_fbk_a[i]);
                dist_b += (joint_goal_b[i] - joint_pos_fbk_b[i]) * (joint_goal_b[i] - joint_pos_fbk_b[i]);
            }
            if (dist_a < 0.0001) {
                ida = nodea->timeStep;
                if (nodea->Type1Next != nullptr) {
                    bool safe = true;
                    auto type2edges = nodea->Type1Next->Type2Prev;
                    for (std::shared_ptr<TPG::type2Edge> edge : type2edges) {
                        if (edge->nodeFrom->timeStep > idb) {
                            safe = false;
                        }
                    }
                    if (safe) {
                        nodea = nodea->Type1Next;
                        joint_goal_a = nodea->pose.joint_values;
                        dta = 0;
                    }
                }
            }
            if (dist_b < 0.0001) {
                idb = nodeb->timeStep;
                if (nodeb->Type1Next != nullptr) {
                    bool safe = true;
                    auto type2edges = nodeb->Type1Next->Type2Prev;
                    for (std::shared_ptr<TPG::type2Edge> edge : type2edges) {
                        if (edge->nodeFrom->timeStep > ida) {
                            safe = false;
                        }
                    }
                    if (safe) {
                        nodeb = nodeb->Type1Next;
                        joint_goal_b = nodeb->pose.joint_values;
                        dtb = 0;
                    }
                }
            }

            // check if the robot is at the goal
            if (nodea->Type1Next == nullptr && nodeb->Type1Next == nullptr) {
                break;
            }

            // move the robot to the current timestamp goal
            for (int i = 0; i < 6; i++) {
                double alpha_a = std::max(1.0, (dta - dt) / dt);
                double alpha_b = std::max(1.0, (dtb - dt) / dt);
                joint_pos_cmd_a[i] = joint_pos_fbk_a[i] + (joint_goal_a[i] - joint_pos_fbk_a[i]) * alpha_a;
                joint_pos_cmd_b[i] = joint_pos_fbk_b[i] + (joint_goal_b[i] - joint_pos_fbk_b[i]) * alpha_b;
            }

            robot_hw_.setJointPosCmd(joint_pos_cmd_a, robot_index_a);
            robot_hw_.setJointPosCmd(joint_pos_cmd_b, robot_index_b);
            dta += 0.004;
            dtb += 0.004;
            usleep(4000);
        }
    }

private:
    HardwareInterface robot_hw_;
    int robot_index_a = 0;
    int robot_index_b = 1;
    double scale_ = 1.0;
    double *joint_pos_cmd_a, *joint_pos_cmd_b;
    double *joint_pos_fbk_a, *joint_pos_fbk_b;
    std::shared_ptr<TPG::TPG> tpg_;
    trajectory_msgs::JointTrajectory traj_;
};

int main(int argc, char *argv[])
{
    // read the adg_path from input
    if (argc < 3) {
        ROS_ERROR("Usage: %s <adg/tpg> <adg_path>", argv[0]);
        return -1;
    }

    std::string type = argv[1];
    std::string adg_path = argv[2]; 

    MotoPlusExecutor executor;

    executor.set_scale(1.5);
    executor.init(adg_path, type);
    //executor.goto_start();
    executor.sync_exec();

    ROS_INFO("Finished execution");

    return true;
}