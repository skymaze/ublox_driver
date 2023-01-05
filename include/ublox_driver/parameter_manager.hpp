/**
* This file is part of ublox-driver.
*
* Copyright (C) 2021 Aerial Robotics Group, Hong Kong University of Science and Technology
* Author: CAO Shaozu (shaozu.cao@gmail.com)
*
* ublox-driver is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ublox-driver is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ublox-driver. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PARAMETER_MANAGER_HPP_
#define PARAMETER_MANAGER_HPP_

#include <string>
#include <fstream>
#include <glog/logging.h>
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

#include "yaml/Yaml.hpp"

typedef std::pair<std::string, std::string> RcvConfigRecord;

class ParameterManager
{
    private:
        ParameterManager() { };
        ~ParameterManager() { };
        ParameterManager(const ParameterManager&);
        ParameterManager& operator=(const ParameterManager&);

        Eigen::MatrixXd parse_matrix(int &rows, int &cols, std::vector<double> &matrix_data)
        {
            Eigen::MatrixXd result;
            assert(rows > 0 && cols > 0);
            result.resize(rows, cols);
            assert(matrix_data.size() == static_cast<size_t>(rows*cols));
            for (int i = 0; i < rows; ++i)
                for (int j = 0; j < cols; ++j)
                    result(i, j) = matrix_data.at(i*cols+j);
            return result;
        }

        // convert relative path to the absolute one
        std::string path_rel2abs(std::string rel_path)
        {
            assert(!rel_path.empty() && "Input relative path cannot be empty.\n");
            if (rel_path[0] == '~')
                rel_path.replace(0, 1, getenv("HOME"));
            char actual_dir[PATH_MAX+1];
            if(!realpath(rel_path.c_str(), actual_dir))
                std::cerr << "ERROR: Failed to obtain the absolute path of " << rel_path << '\n';
            return std::string(actual_dir);
        }

    public:
        static ParameterManager& getInstance() 
            {
            static ParameterManager instance;
            return instance;
        }

        void read_parameter(rclcpp::Node::SharedPtr & node)
        {
            node->declare_parameter("online", true);
            online = node->get_parameter("online").get_parameter_value().get<bool>();
            node->declare_parameter("input_rtcm", false);
            input_rtcm = node->get_parameter("input_rtcm").get_parameter_value().get<bool>();
            node->declare_parameter("to_ros", true);
            to_ros = node->get_parameter("to_ros").get_parameter_value().get<bool>();
            node->declare_parameter("to_file", false);
            to_file = node->get_parameter("to_file").get_parameter_value().get<bool>();
            node->declare_parameter("to_serial", false);
            to_serial = node->get_parameter("to_serial").get_parameter_value().get<bool>();
            node->declare_parameter("config_receiver_at_start", false);
            config_receiver_at_start = node->get_parameter("config_receiver_at_start").get_parameter_value().get<bool>();

            if (online)
            {
                node->declare_parameter("input_serial_port", "/dev/ttyACM0");
                input_serial_port = node->get_parameter("input_serial_port").get_parameter_value().get<std::string>();
                node->declare_parameter("serial_baud_rate", 921600);
                serial_baud_rate = node->get_parameter("serial_baud_rate").get_parameter_value().get<long>();
                node->declare_parameter("rtcm_tcp_port", 3503);
                rtcm_tcp_port = node->get_parameter("rtcm_tcp_port").get_parameter_value().get<uint64_t>();
            }
            else
            {
                node->declare_parameter("ubx_filepath", "~/tmp/ublox_driver_test/test.ubx");
                ubx_filepath = node->get_parameter("ubx_filepath").get_parameter_value().get<std::string>();
            }
            if (to_serial) {
                node->declare_parameter("output_serial_port", "/dev/ttyACM0");
                output_serial_port = node->get_parameter("output_serial_port").get_parameter_value().get<std::string>();
            }
            if (to_file) {
                node->declare_parameter("dump_dir", "~/tmp/ublox_driver_test/");
                dump_dir = node->get_parameter("dump_dir").get_parameter_value().get<std::string>();
            }

            node->declare_parameter("rtk_correction_ecef", std::vector<double>({0.0, 0.0, 0.0}));
            std::vector<double> matrix_data = node->get_parameter("rtk_correction_ecef").get_parameter_value().get<std::vector<double>>();
            assert(matrix_data.size() == 3);
            rtk_correction_ecef[0] = matrix_data[0];
            rtk_correction_ecef[1] = matrix_data[1];
            rtk_correction_ecef[2] = matrix_data[2];

            if (online && config_receiver_at_start)
            {
                node->declare_parameter("receiver_config_filepath", "~/ros2_ws/src/ublox_driver/config/ublox_rcv_config.yaml");
                std::string rcv_config_filepath = node->get_parameter("receiver_config_filepath").get_parameter_value().get<std::string>();
                if (!rcv_config_filepath.empty())
                {
                    Yaml::Node rcv_config_root;
                    Yaml::Parse(rcv_config_root, rcv_config_filepath.c_str());
                    for (Yaml::Iterator it = rcv_config_root.Begin(); it != rcv_config_root.End(); it++)
                        receiver_configs.emplace_back((*it).first, (*it).second.As<std::string>());
                }
            }
        }

        bool online, input_rtcm;
        bool to_ros, to_file, to_serial;
        bool config_receiver_at_start;
        std::string ubx_filepath;
        std::string output_serial_port;
        std::string input_serial_port;
        long serial_baud_rate;
        uint64_t rtcm_tcp_port;
        std::string dump_dir;
        Eigen::Vector3d rtk_correction_ecef;
        std::vector<RcvConfigRecord> receiver_configs;
        constexpr static uint32_t MSG_HEADER_LEN = 6;
        constexpr static uint32_t IO_TIMEOUT_MS = 50;
};

#endif