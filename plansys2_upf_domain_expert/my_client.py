from plansys2_msgs import srv
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
import rclpy
from rclpy.node import Node

from typing import List


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')

        self.get_domain_client = self.create_client(
            srv.GetDomain, 'domain_expert_upf/get_domain')
        self.get_name_client = self.create_client(
            srv.GetDomainName, 'domain_expert_upf/get_domain_name')
        self.get_types_client = self.create_client(
            srv.GetDomainTypes, 'domain_expert_upf/get_domain_types')
        self.get_constants_client = self.create_client(
            srv.GetDomainConstants, 'domain_expert_upf/get_domain_constants')
        self.get_actions_client = self.create_client(
            srv.GetDomainActions, 'domain_expert_upf/get_domain_actions')
        self.get_action_details_client = self.create_client(
            srv.GetDomainActionDetails, 'domain_expert_upf/get_domain_action_details')
        self.get_durative_actions_client = self.create_client(
            srv.GetDomainActions, 'domain_expert_upf/get_domain_durative_actions')
        self.get_durative_action_details_client = self.create_client(
            srv.GetDomainDurativeActionDetails, 'domain_expert_upf/get_domain_durative_action_details')
        self.get_predicates_client = self.create_client(
            srv.GetStates, 'domain_expert_upf/get_domain_predicates')
        self.get_predicate_details_client = self.create_client(
            srv.GetNodeDetails, 'domain_expert_upf/get_domain_predicate_details')
        self.get_functions_client = self.create_client(
            srv.GetStates, 'domain_expert_upf/get_domain_functions')
        self.get_function_details_client = self.create_client(
            srv.GetNodeDetails, 'domain_expert_upf/get_domain_function_details')

        self.get_domain_client_cpp = self.create_client(
            srv.GetDomain, 'domain_expert/get_domain')
        self.get_name_client_cpp = self.create_client(
            srv.GetDomainName, 'domain_expert/get_domain_name')
        self.get_types_client_cpp = self.create_client(
            srv.GetDomainTypes, 'domain_expert/get_domain_types')
        self.get_constants_client_cpp = self.create_client(
            srv.GetDomainConstants, 'domain_expert/get_domain_constants')
        self.get_actions_client_cpp = self.create_client(
            srv.GetDomainActions, 'domain_expert/get_domain_actions')
        self.get_action_details_client_cpp = self.create_client(
            srv.GetDomainActionDetails, 'domain_expert/get_domain_action_details')
        self.get_durative_actions_client_cpp = self.create_client(
            srv.GetDomainActions, 'domain_expert/get_domain_durative_actions')
        self.get_durative_action_details_client_cpp = self.create_client(
            srv.GetDomainDurativeActionDetails, 'domain_expert/get_domain_durative_action_details')
        self.get_predicates_client_cpp = self.create_client(
            srv.GetStates, 'domain_expert/get_domain_predicates')
        self.get_predicate_details_client_cpp = self.create_client(
            srv.GetNodeDetails, 'domain_expert/get_domain_predicate_details')
        self.get_functions_client_cpp = self.create_client(
            srv.GetStates, 'domain_expert/get_domain_functions')
        self.get_function_details_client_cpp = self.create_client(
            srv.GetNodeDetails, 'domain_expert/get_domain_function_details')

        self.de_get_state_service = self.create_client(
            GetState, 'domain_expert_upf/get_state')
        self.de_change_state_service = self.create_client(
            ChangeState, 'domain_expert_upf/change_state')
        self.de_get_state_service_cpp = self.create_client(
            GetState, 'domain_expert/get_state')
        self.de_change_state_service_cpp = self.create_client(
            ChangeState, 'domain_expert/change_state')

        self.get_state()
        self.change_state(Transition.TRANSITION_CONFIGURE)
        self.get_state()
        self.change_state(Transition.TRANSITION_ACTIVATE)
        self.get_state()

    def get_state(self):
        request = GetState.Request()

        while not self.de_get_state_service_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        future = self.de_get_state_service_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(f"response: {response}")

        while not self.de_get_state_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')

        future = self.de_get_state_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(f"response2: {response}")

    def change_state(self, transition):
        request = ChangeState.Request()
        request.transition.id = transition

        while not self.de_change_state_service_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        future = self.de_change_state_service_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(f"response: {response}")

        while not self.de_change_state_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')

        future = self.de_change_state_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(f"response2: {response}")

    def getDomain(self):
        while not self.get_domain_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = srv.GetDomain.Request()
        future = self.get_domain_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.get_domain_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')
        future = self.get_domain_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()

        if response.domain != response_cpp.domain:
            print("========================================")
            print(response.domain)
            print("========================================")
            print(response_cpp.domain)
            print("========================================")
            print("error getDomain")
        else:
            print(response.domain)

        return response.domain

    def getName(self) -> str:
        while not self.get_name_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = srv.GetDomainName.Request()
        future = self.get_name_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.get_name_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')
        future = self.get_name_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()
        if response.name != response_cpp.name:
            print("========================================")
            print(response.name)
            print("========================================")
            print(response_cpp.name)
            print("========================================")
            print("error getName")
        else:
            print(response.name)

        return response.name

    def getTypes(self):
        while not self.get_types_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = srv.GetDomainTypes.Request()
        future = self.get_types_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.get_types_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')
        future = self.get_types_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()
        if response.types != response_cpp.types:
            print("========================================")
            print(response.types)
            print("========================================")
            print(response_cpp.types)
            print("========================================")
            print("error get_types")
        else:
            print(response.types)

        return response.types

    def getConstants(self, type: str):
        while not self.get_constants_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = srv.GetDomainConstants.Request()
        request.type = type
        future = self.get_constants_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.get_constants_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')
        future = self.get_constants_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()
        if response.constants != response_cpp.constants:
            print("========================================")
            print(response.constants)
            print("========================================")
            print(response_cpp.constants)
            print("========================================")
            print("error get_constants")
        else:
            print(response.constants)

        return response.constants

    def getActions(self):
        while not self.get_actions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = srv.GetDomainActions.Request()
        future = self.get_actions_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.get_actions_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')
        future = self.get_actions_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()
        if response.actions != response_cpp.actions:
            print("========================================")
            print(response.actions)
            print("========================================")
            print(response_cpp.actions)
            print("========================================")
            print("error get_actions")
        else:
            print(response.actions)

        return response.actions

    def getAction(self, action: str, parameters: List[str]):
        while not self.get_action_details_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = srv.GetDomainActionDetails.Request()
        request.action = action
        request.parameters = parameters
        future = self.get_action_details_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.get_action_details_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')
        future = self.get_action_details_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()
        if response.action != response_cpp.action:
            print("========================================")
            print(response.action)
            print("========================================")
            print(response_cpp.action)
            print("========================================")
            print("error get_action_details")
        else:
            print(response.action)

        return response.action

    def getDurativeActions(self):
        while not self.get_durative_actions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = srv.GetDomainActions.Request()
        future = self.get_durative_actions_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.get_durative_actions_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')
        future = self.get_durative_actions_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()
        if response.actions != response_cpp.actions:
            print("========================================")
            print(response.actions)
            print("========================================")
            print(response_cpp.actions)
            print("========================================")
            print("error get_durative_actions")
        else:
            print(response.actions)

        return response.actions

    def getDurativeAction(self, action: str, parameters: List[str]):
        while not self.get_durative_action_details_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = srv.GetDomainDurativeActionDetails.Request()
        request.durative_action = action
        request.parameters = parameters
        future = self.get_durative_action_details_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.get_durative_action_details_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')
        future = self.get_durative_action_details_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()
        if response.durative_action != response_cpp.durative_action:
            print("========================================")
            # print(f"at_start_requirements: {response.durative_action.at_start_requirements}")
            # print(f"at_end_requirements: {response.durative_action.at_end_requirements}")
            # print(f"over_all_requirements: {response.durative_action.over_all_requirements}")
            print(f"at_start_effects: {response.durative_action.at_start_effects}")
            print(f"at_end_effects: {response.durative_action.at_end_effects}")
            print("========================================")
            # print(f"at_start_requirements: {response_cpp.durative_action.at_start_requirements}")
            # print(f"at_end_requirements: {response_cpp.durative_action.at_end_requirements}")
            # print(f"over_all_requirements: {response_cpp.durative_action.over_all_requirements}")
            print(f"at_start_effects: {response_cpp.durative_action.at_start_effects}")
            print(f"at_end_effects: {response_cpp.durative_action.at_end_effects}")
            print("========================================")
            print("error get_durative_action_details")
        else:
            print(response.durative_action)

        return response.durative_action

    def getPredicates(self):
        while not self.get_predicates_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = srv.GetStates.Request()
        future = self.get_predicates_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.get_predicates_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')
        future = self.get_predicates_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()
        if response.states != response_cpp.states:
            print("========================================")
            print(response.states)
            print("========================================")
            print(response_cpp.states)
            print("========================================")
            print("error get_predicates")
        else:
            print(response.states)

        return response.states

    def getPredicate(self, predicate: str):
        while not self.get_predicate_details_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = srv.GetNodeDetails.Request()
        request.expression = predicate
        future = self.get_predicate_details_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.get_predicate_details_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')
        future = self.get_predicate_details_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()
        if response.node != response_cpp.node:
            print("========================================")
            print(response.node)
            print("========================================")
            print(response_cpp.node)
            print("========================================")
            print("error get_predicate_details")
        else:
            print(response.node)

        return response.node

    def getFunctions(self):
        while not self.get_functions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = srv.GetStates.Request()
        future = self.get_functions_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.get_functions_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')
        future = self.get_functions_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()
        if response.states != response_cpp.states:
            print("========================================")
            print(response.states)
            print("========================================")
            print(response_cpp.states)
            print("========================================")
            print("error get_functions")
        else:
            print(response.states)

        return response.states

    def getFunction(self, function: str):
        while not self.get_function_details_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = srv.GetNodeDetails.Request()
        request.expression = function
        future = self.get_function_details_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while not self.get_function_details_client_cpp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')
        future = self.get_function_details_client_cpp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response_cpp = future.result()
        if response.node != response_cpp.node:
            print("========================================")
            print(response.node)
            print("========================================")
            print(response_cpp.node)
            print("========================================")
            print("error get_function_details")
        else:
            print(response.node)

        return response.node


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()

    # minimal_client.get_logger().info(
    #     f'Domain: {minimal_client.getDomain()}')
    # minimal_client.get_logger().info(
    #     f'Domain name: {minimal_client.getName()}')
    # minimal_client.get_logger().info(
    #     f'Domain types: {minimal_client.getTypes()}')
    # minimal_client.get_logger().info(
    #     f'Domain constants: {minimal_client.getConstants("counter")}')
    # minimal_client.get_logger().info(
    #     f'Domain actions: {minimal_client.getActions()}')
    # minimal_client.get_logger().info(
    #     f'Domain action details: {minimal_client.getAction("test_action1", [""])}') # TODO: params list
    # minimal_client.get_logger().info(
    #     f'Domain durative actions: {minimal_client.getDurativeActions()}')
    # minimal_client.get_logger().info(
    #     f'Domain durative action details: {minimal_client.getDurativeAction("test_durative_action", [""])}') # TODO: params list
    # minimal_client.get_logger().info(
    #     f'Domain predicates: {minimal_client.getPredicates()}')
    # minimal_client.get_logger().info(
    #     f'Domain predicate details: {minimal_client.getPredicate("test_predicate")}')
    # minimal_client.get_logger().info(
    #     f'Domain predicate details: {minimal_client.getPredicate("test_predicate2")}')
    # minimal_client.get_logger().info(
    #     f'Domain functions: {minimal_client.getFunctions()}')
    # minimal_client.get_logger().info(
    #     f'Domain function details: {minimal_client.getFunction("value")}')
    # minimal_client.get_logger().info(
    #     f'Domain function details: {minimal_client.getFunction("value2")}')

    # minimal_client.getDomain()
    # minimal_client.getName()
    # minimal_client.getTypes()
    # minimal_client.getConstants("counter")
    # minimal_client.getActions()
    # minimal_client.getAction("test_action", [])
    # minimal_client.getDurativeActions()
    # minimal_client.getDurativeAction("move", [])
    # minimal_client.getPredicates()
    # minimal_client.getPredicate("robot_at")
    # minimal_client.getFunctions()
    # minimal_client.getFunction("test_fn")

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
