


#include <ros/ros.h>
#include <string>
#include <Eigen/Core>





class DataParser
{
public:
    DataParser(const ros::NodeHandle & nh): nh_(nh){}

    const ros::NodeHandle nh_;

	template<typename T>
	void parse_input_argument(const std::string & var_name, T & var_out);
	void parse_input_vector(const std::string & vec_name, Eigen::VectorXd & vec_out);

};
