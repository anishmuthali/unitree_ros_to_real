

#include "data_parser.hpp"

template<typename T>
void DataParser::parse_input_argument(const std::string & var_name, T & var_out){

    // http://wiki.ros.org/roscpp/Overview/Names%20and%20Node%20Information#Manipulating_Names
    // http://docs.ros.org/en/melodic/api/roscpp/html/classros_1_1NodeHandle.html

    // std::string this_node;
    // this_node = ros::this_node::getName();
    // std::cout << "this_node: " << this_node << "\n";

    // std::string resolved_name;
    // resolved_name = this->nh.resolveName(var_name); // Resolve a name using the NodeHandle's namespace. Returns ns+var_name; ns="/" by default, can be overridden in the launch file
    // std::cout << "resolved_name: " << resolved_name << "\n";


    // if(this->nh.getParam(var_name,var)){
    // std::string name_param_full("/"+node_name+"/"+var_name);
    std::string name_param_full = ros::this_node::getName() + this->nh.resolveName(var_name);
    if(this->nh.getParam(name_param_full,var_out)){
        std::cout << name_param_full << ": " << var_out << "\n";
    }
    else{
        std::cout << name_param_full << " failed to load because it doesn't exist...\n";
    }

    return;
}

void DataParser::parse_input_vector(const std::string & vec_name, Eigen::VectorXd & vec_out){

    // http://wiki.ros.org/roscpp/Overview/Names%20and%20Node%20Information#Manipulating_Names
    // http://docs.ros.org/en/melodic/api/roscpp/html/classros_1_1NodeHandle.html

    std::string name_param_full = ros::this_node::getName() + this->nh.resolveName(vec_name);
    Eigen::IOFormat clean_format = Eigen::IOFormat(4, 0, ", ", "\n", "[", "]");

    std::vector<double> vec_doubles;
    if(this->nh.getParam(name_param_full,vec_doubles)){
        vec_out = Eigen::Map<Eigen::VectorXd>(vec_doubles.data(), vec_doubles.size());
        std::cout << name_param_full << ": " << vec_out.transpose().format(clean_format) << "\n";
    }
    else{
        std::cout << name_param_full << " failed to load because it doesn't exist...\n";
    }

    return;
}
