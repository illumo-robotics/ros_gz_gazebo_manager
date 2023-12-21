/*******************************************************************************
 *  Copyright (c) Gezp (https://github.com/gezp), All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/convert_decl.hpp>

namespace ros_gz_bridge
{

template<typename ROS_SERVICE_T, typename IGN_REQUEST_T, typename IGN_RESPONSE_T>
class GzServiceBrigde{
   public:
    GzServiceBrigde(rclcpp::Node::SharedPtr ros_node,
                    std::shared_ptr<gz::transport::Node> gz_node, 
                    const std::string & ros_service_name,
                    const std::string & gz_service_name,
                    int gz_timeout=5000) 
    : ros_node_(ros_node),gz_node_(gz_node),
    gz_service_name_(gz_service_name),
    gz_timeout_(gz_timeout) 
    {
        //ros srv
        using std::placeholders::_1;
        using std::placeholders::_2;
        ros_srv_ = ros_node_->create_service<ROS_SERVICE_T>(ros_service_name,
            std::bind(&GzServiceBrigde::ros_callback, this, _1,_2));
    }
    ~GzServiceBrigde(){};

    void ros_callback(const std::shared_ptr<typename ROS_SERVICE_T::Request> request,
                      const std::shared_ptr<typename ROS_SERVICE_T::Response> response)
    { 
        // Request message
        IGN_REQUEST_T req;
        convert_ros_to_gz(*request,req);
        IGN_RESPONSE_T rep;
        bool result;
        bool executed = gz_node_->Request(gz_service_name_, req, gz_timeout_, rep, result);
        if (executed) {
            if (result) {
                convert_gz_to_ros(rep,*response);
            }else{
                RCLCPP_ERROR(ros_node_->get_logger(), "Ignition Service %s call failed\n %s",
                    gz_service_name_.c_str(), req.DebugString().c_str());  
            }
        }else{
            RCLCPP_ERROR(ros_node_->get_logger(), "Ignition Service %s call timed out\n %s",
                gz_service_name_.c_str(), req.DebugString().c_str());  
        } 
    } 
    private:
        rclcpp::Node::SharedPtr ros_node_;
        std::shared_ptr<gz::transport::Node> gz_node_;
        typename rclcpp::Service<ROS_SERVICE_T>::SharedPtr ros_srv_;
        std::string gz_service_name_;
        int gz_timeout_;
};


}