/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include <chrono>
#include <boost/date_time.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <ros/ros.h>
#include <deterministic_trac_ik/deterministic_trac_ik.hpp>
#include <deterministic_trac_ik/utils.h>

void test(
    ros::NodeHandle& nh,
    int num_samples,
    const std::string& chain_start,
    const std::string& chain_end,
    int max_iterations,
    const std::string& urdf_param)
{
    double eps = 1e-5;

    const std::string& robot_description = "robot_description";
    const std::string& base_name = chain_start;
    const std::string& tip_name = chain_end;

    ros::NodeHandle node_handle("~");
    urdf::Model robot_model;

    if (!Deterministic_TRAC_IK::LoadModelOverride(node_handle, robot_description, robot_model)) {
        ROS_WARN_STREAM_NAMED("deterministic_trac_ik", "Failed to load robot model");
        return;
    }

    ROS_DEBUG_STREAM_NAMED("deterministic_trac_ik","Reading joints and links from URDF");

    KDL::Chain chain;
    std::vector<std::string> link_names;
    std::vector<std::string> joint_names;
    KDL::JntArray joint_min;
    KDL::JntArray joint_max;
    if (!Deterministic_TRAC_IK::InitKDLChain(
        robot_model, base_name, tip_name,
        chain, link_names, joint_names, joint_min, joint_max))
    {
        ROS_WARN_STREAM_NAMED("deterministic_trac_ik", "Failed to initialize KDL chain");
        return;
    }

    ROS_INFO("link names: %zu", link_names.size());
    ROS_INFO("joint names: %zu", joint_names.size());
    ROS_INFO("joint_min: %zu", joint_min.data.size());
    ROS_INFO("joint_max: %zu", joint_max.data.size());

    Deterministic_TRAC_IK::Deterministic_TRAC_IK tracik_solver(
            chain, joint_min, joint_max, max_iterations, eps, Deterministic_TRAC_IK::Speed);

    auto& ll = tracik_solver.getLowerLimits();
    auto& ul = tracik_solver.getUpperLimits();

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    // Set up KDL IK
    KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
    KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
    KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, max_iterations, eps); // Joint Limit Solver
    // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)

    ////////////////////////////////////////////////////////////////////////
    // Create Nominal chain configuration midway between all joint limits //
    ////////////////////////////////////////////////////////////////////////

    KDL::JntArray nominal(chain.getNrOfJoints());

    for (uint j = 0; j < nominal.data.size(); ++j) {
        nominal(j) = 0.5 * (ll(j) + ul(j));
    }

    /////////////////////////////////////////////////////////////////
    // Create desired number of valid, random joint configurations //
    /////////////////////////////////////////////////////////////////

    std::vector<KDL::JntArray> position_samples;
    KDL::JntArray q(chain.getNrOfJoints());

    std::default_random_engine rng;

    for (int i = 0; i < num_samples; ++i) {
        for (uint j = 0; j < ll.data.size(); ++j) {
            std::uniform_real_distribution<double> dist(ll(j), ul(j));
            q(j) = dist(rng);
        }
        position_samples.push_back(q);
    }

    KDL::JntArray result;

    double total_time = 0.0;
    int success = 0;

    ROS_INFO_STREAM("*** Testing KDL with " << num_samples << " random samples");

    int tens = -1;

    for (int i = 0; i < num_samples; i++) {
        KDL::Frame end_effector_pose;
        fk_solver.JntToCart(position_samples[i], end_effector_pose);

        auto before = std::chrono::high_resolution_clock::now();
        int rc = kdl_solver.CartToJnt(nominal, end_effector_pose, result);
        auto after = std::chrono::high_resolution_clock::now();

        // accumulate time taken
        auto diff = std::chrono::duration<double>(after - before);
        total_time += diff.count();

        // count success
        if (rc >= 0) {
            ++success;
        }

        // log progress
        int new_tens = i * 10 / num_samples;
        if (new_tens != tens) {
            ROS_DEBUG("%d%% done", i * 100 / num_samples);
            tens = new_tens;
        }
    }

    ROS_INFO_STREAM("KDL found " << success << " solutions (" << 100.0 * success / num_samples << "\%) with an average of " << total_time / num_samples << " secs per sample");

    tens = -1;
    total_time = 0.0;
    success = 0;

    ROS_INFO_STREAM("*** Testing TRAC-IK with " << num_samples << " random samples");

    for (int i = 0; i < num_samples; i++) {
        KDL::Frame end_effector_pose;
        fk_solver.JntToCart(position_samples[i], end_effector_pose);

        auto before = std::chrono::high_resolution_clock::now();
        int rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
        auto after = std::chrono::high_resolution_clock::now();

        auto diff = std::chrono::duration<double>(after - before);
        total_time += diff.count();

        if (rc >= 0) {
            success++;

            KDL::Frame result_pose;
            fk_solver.JntToCart(result, result_pose);

            if (!Equal(end_effector_pose, result_pose, 1e-3)) {
                ROS_WARN_STREAM("Ik is bad and should feel bad (" << end_effector_pose << " vs " << result_pose);
            }
        }

        int new_tens = i * 10 / num_samples;
        if (new_tens != tens) {
            ROS_DEBUG("%d%% done", i * 100 / num_samples);
            tens = new_tens;
        }
    }

    ROS_INFO_STREAM("TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples << "\%) with an average of " << total_time / num_samples << " secs per sample");
}

int main(int argc, char** argv)
{
    srand(1);
    ros::init(argc, argv, "ik_tests");
    ros::NodeHandle nh("~");

    int num_samples;
    std::string chain_start;
    std::string chain_end;
    std::string urdf_param;
    int max_iterations;

    nh.param("num_samples", num_samples, 1000);
    nh.param("chain_start", chain_start, std::string(""));
    nh.param("chain_end", chain_end, std::string(""));

    if (chain_start=="" || chain_end=="") {
        ROS_FATAL("Missing chain info in launch file");
        exit (-1);
    }

    nh.param("max_iterations", max_iterations, 100);
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));

    if (num_samples < 1) {
        num_samples = 1;
    }

    test(nh, num_samples, chain_start, chain_end, max_iterations, urdf_param);

    // Useful when you make a script that loops over multiple launch files that test different robot chains
    // std::vector<char *> commandVector;
    // commandVector.push_back((char*)"killall");
    // commandVector.push_back((char*)"-9");
    // commandVector.push_back((char*)"roslaunch");
    // commandVector.push_back(NULL);

    // char **command = &commandVector[0];
    // execvp(command[0],command);

    return 0;
}
