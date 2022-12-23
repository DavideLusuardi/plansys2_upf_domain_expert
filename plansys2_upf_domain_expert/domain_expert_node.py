from plansys2_msgs import srv
from plansys2_upf_domain_expert import DomainExpert

import rclpy
from rclpy.node import Node

# TODO: manage errors

class DomainExpertNode(Node):

    def __init__(self):
        super().__init__('domain_expert')
        # self.domain_expert = DomainExpert('/home/davide/plansys2_ws/src/plansys2_upf_domain_expert/tmp/domain.pddl')
        self.domain_expert = DomainExpert('/home/davide/plansys2_ws/src/plansys2_upf_domain_expert/tmp/domain2.pddl')
        # self.domain_expert = DomainExpert('/home/davide/plansys2_ws/src/ros2_planning_system/plansys2_domain_expert/test/pddl/domain_simple.pddl')

        self.create_service(srv.GetDomain, 'domain_expert_upf/get_domain',
                            self.get_domain_service_callback)
        self.create_service(srv.GetDomainName, 'domain_expert_upf/get_domain_name',
                            self.get_domain_name_service_callback)
        self.create_service(srv.GetDomainTypes, 'domain_expert_upf/get_domain_types',
                            self.get_domain_types_service_callback)
        self.create_service(srv.GetDomainConstants, 'domain_expert_upf/get_domain_constants',
                            self.get_domain_constants_service_callback)
        self.create_service(srv.GetDomainActions, 'domain_expert_upf/get_domain_actions',
                            self.get_domain_actions_service_callback)
        self.create_service(srv.GetDomainActionDetails, 'domain_expert_upf/get_domain_action_details',
                            self.get_domain_action_details_service_callback)
        self.create_service(srv.GetDomainActions, 'domain_expert_upf/get_domain_durative_actions',
                            self.get_domain_durative_actions_service_callback)
        self.create_service(srv.GetDomainDurativeActionDetails, 'domain_expert_upf/get_domain_durative_action_details',
                            self.get_domain_durative_action_details_service_callback)
        self.create_service(srv.GetStates, 'domain_expert_upf/get_domain_predicates',
                            self.get_domain_predicates_service_callback)
        self.create_service(srv.GetNodeDetails, 'domain_expert_upf/get_domain_predicate_details',
                            self.get_domain_predicate_details_service_callback)
        self.create_service(srv.GetStates, 'domain_expert_upf/get_domain_functions',
                            self.get_domain_functions_service_callback)
        self.create_service(srv.GetNodeDetails, 'domain_expert_upf/get_domain_function_details',
                            self.get_domain_function_details_service_callback)

    def get_domain_service_callback(self, request, response):
        self.get_logger().info(f'get_domain::Incoming request: {request}')
        response.domain = self.domain_expert.getDomain()
        response.success = True

        return response

    def get_domain_name_service_callback(self, request, response):
        self.get_logger().info(f'get_domain_name::Incoming request: {request}')
        response.name = self.domain_expert.getName()
        response.success = True

        return response

    def get_domain_types_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_types::Incoming request: {request}')
        response.types = self.domain_expert.getTypes()
        response.success = True

        return response

    def get_domain_constants_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_constants::Incoming request: {request}')
        response.constants = self.domain_expert.getConstants(request.type)
        response.success = True

        return response

    def get_domain_actions_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_actions::Incoming request: {request}')
        response.actions = self.domain_expert.getActions()
        response.success = True

        return response

    def get_domain_action_details_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_action_details::Incoming request: {request}')
        action = self.domain_expert.getAction(request.action, request.parameters)
        if action is None:
            self.get_logger().warn(
                f'Requesting a non-existing action [{request.action}]')
            response.success = False
            response.error_info = "Action not found"
        else:
            response.action = action
            response.success = True

        return response

    def get_domain_durative_actions_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_durative_actions::Incoming request: {request}')
        response.actions = self.domain_expert.getDurativeActions()
        response.success = True

        return response

    def get_domain_durative_action_details_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_durative_action_details::Incoming request: {request}')
        response.durative_action = self.domain_expert.getDurativeAction(
            request.durative_action, request.parameters)
        response.success = True

        return response

    def get_domain_predicates_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_predicates::Incoming request: {request}')
        response.states = self.domain_expert.getPredicates()
        response.success = True

        return response

    def get_domain_predicate_details_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_predicate_details::Incoming request: {request}')
        predicate = self.domain_expert.getPredicate(request.expression)
        if predicate is None:
            self.get_logger().warn(
                f'Requesting a non-existing predicate [{request.expression}]')
            response.success = False
            response.error_info = "Predicate not found"
        else:
            response.node = predicate
            response.success = True

        return response

    def get_domain_functions_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_functions::Incoming request: {request}')
        response.states = self.domain_expert.getFunctions()
        response.success = True

        return response

    def get_domain_function_details_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_function_details::Incoming request: {request}')
        func = self.domain_expert.getFunction(request.expression)
        if func is None:
            self.get_logger().warn(
                f'Requesting a non-existing function [{request.expression}]')
            response.success = False
            response.error_info = "Function not found"
        else:
            response.node = func
            response.success = True

        return response


def main():
    rclpy.init()

    domain_expert_node = DomainExpertNode()

    rclpy.spin(domain_expert_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
