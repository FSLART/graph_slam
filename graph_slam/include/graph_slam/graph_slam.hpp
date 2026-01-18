#ifndef GRAPH_SLAM_H_
#define GRAPH_SLAM_H_

#include "rclcpp/rclcpp.hpp"
#include "graph_slam/associationSolver.hpp"

#define ASSICIATION_MODE 1
#define CONES_TOPIC "/mapping/cones"

class GraphSLAM : public rclcpp::Node
{
public:
    GraphSLAM();
    ~GraphSLAM();

    void observations_callback(const lart_msgs::msg::ConeArray::SharedPtr msg);

private:
    rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr observations_subscriber_;

protected:
    AssociationSolver *association_solver_;
};

#endif