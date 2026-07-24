#include <chrono>
#include <cstdlib>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include "chapt4_interfaces/srv/test.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

using namespace std::chrono_literals;
using Patrol=chapt4_interfaces::srv::Patrol;
using SetP=rcl_interfaces::srv::SetParameters;

class PatrolClient :public rclcpp::Node
{
public:
    PatrolClient():Node("patrol_client")
    {
        patrol_client_=this->create_client<Patrol>("patrol");
        timer_=this->create_wall_timer(10s,std::bind(&PatrolClient::timer_callback,this));
        srand(time(NULL));
    }

    void update_server_param_k(double k)
    {
        auto param=rcl_interfaces::msg::Parameter();
        param.name="k";
        auto param_value=rcl_interfaces::msg::ParameterValue();
        param_value.type=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_value.double_value=k;
        param_value=param_value;

        auto response=call_set_parameters(param);
        if(response==nullptr)
        {
            RCLCPP_WARN(this->get_logger(),"param change fail");
        }

        else{
            for(auto result:response->results)
            {   
                if(result.successful)
                {
                    RCLCPP_INFO(this->get_logger(),"param k had be change: %f",k);
                    



                }
                else
                {
                    RCLCPP_INFO(this->get_logger(),"param k fail:%s",result);



                }

            }

        }
    }

    std::shared_ptr<SetP::Response> call_set_parameters(
        rcl_interfaces::msg::Parameter &parameter)
        {
            auto param_client=this->create_client<SetP>("/turtle_controller/set_parameters");
            while(!param_client->wait_for_service(std::chrono::seconds(1)))
            {

                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(),"waitting service but be broken");
                    return nullptr;

                }
                RCLCPP_INFO(this->get_logger(),"wait for server ");


            }

            auto request =std::make_shared<SetP::Request>();
            request->parameters.push_back(parameter);

            auto future=param_client->async_send_request(request);
            rclcpp::spin_until_future_complete(this->get_node_base_interface(),future);
            auto response=future.get();
            return response;

        }

        void timer_callback()
        {

            while (!patrol_client_->wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(),"The service was interrupted during the wait");

                }
                RCLCPP_INFO(this->get_logger(),"Waiting for the server to come online");
            }
            

            auto request =std::make_shared<Patrol::Request>();
            request->target_x=rand() % 15;
            request->target_y=rand() % 15;
            RCLCPP_INFO(this->get_logger(),"reuqest: (%f,%f)",request->target_x,request->target_y);

            patrol_client_->async_send_request(
                request,
                [&](rclcpp::Client<Patrol>::SharedFuture result_future)->void
                {
                    auto response=result_future.get();
                    if(response->result==Patrol::Response::SUCCESS)
                    {
                        RCLCPP_INFO(this->get_logger(),"success");

                    }
                    else if(response->result==Patrol::Response::FAIL)
                    {
                        RCLCPP_INFO(this->get_logger(),"fail");


                    }
                });

            }

    

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;


};



int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<PatrolClient>();
    node->update_server_param_k(1.5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
