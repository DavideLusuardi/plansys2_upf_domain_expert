# UPF Domain Expert

The UPF Domain Expert module is responsible for maintaining the PDDL domain. 

The main class is [`DomainExpertNode`](plansys2_upf_domain_expert/domain_expert_node.py). `DomainExpertNode` simulates a `rclcpp_lifecycle::LifecycleNode` and in its configuration phase reads the `model_file` parameter, which contains the .pddl file from which to read the model.

The class responsible for maintaining this domain is [`DomainExpert`](plansys2_upf_domain_expert/domain_expert.py), which is independent of ROS2.

The UPF Domain Expert does not change while active, accessing its functionality through ROS2 services.

## Services:

- `/domain_expert/get_domain` [`plansys2_msgs::srv::GetDomain`]
- `/domain_expert/get_domain_name` [`plansys2_msgs::srv::GetDomainName`]
- `/domain_expert/get_domain_action_details` [`plansys2_msgs::srv::GetDomainActionDetails`]
- `/domain_expert/get_domain_actions` [`plansys2_msgs::srv::GetDomainActions`]
- `/domain_expert/get_domain_durative_action_details` [`plansys2_msgs::srv::GetDomainDurativeActionDetails`]
- `/domain_expert/get_domain_durative_actions` [`plansys2_msgs::srv::GetDomainActions`]
- `/domain_expert/get_domain_function_details` [`plansys2_msgs::srv::GetNodeDetails`]
- `/domain_expert/get_domain_functions` [`plansys2_msgs::srv::GetStates`]
- `/domain_expert/get_domain_predicate_details` [`plansys2_msgs::srv::GetNodeDetails`]
- `/domain_expert/get_domain_predicates` [`plansys2_msgs::srv::GetStates`]
- `/domain_expert/get_domain_types` [`plansys2_msgs::srv::GetDomainTypes`]
- `/domain_expert/get_domain_constants` [`plansys2_msgs::srv::GetDomainConstants`]