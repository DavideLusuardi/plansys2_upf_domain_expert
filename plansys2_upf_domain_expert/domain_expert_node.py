from plansys2_msgs import srv
from plansys2_upf_domain_expert import DomainExpert
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State

import rclpy
from rclpy.node import Node


class DomainExpertNode(Node):

    def __init__(self):
        super().__init__('domain_expert_upf')

        self.domain_expert = None
        self.state = State.PRIMARY_STATE_UNCONFIGURED
        self.declare_parameter("model_file", "")

        self.create_service(GetState, 'domain_expert_upf/get_state',
                            self.get_state_service_callback)
        self.create_service(ChangeState, 'domain_expert_upf/change_state',
                            self.change_state_service_callback)

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

    def get_state_service_callback(self, request, response):
        response.current_state = State()
        response.current_state.id = self.state
        # response.current_state.label = "" # TODO
        return response

    def change_state_service_callback(self, request, response):
        transition_callback = {
            Transition.TRANSITION_CONFIGURE: self.on_configure,
            Transition.TRANSITION_ACTIVATE: self.on_activate,
            Transition.TRANSITION_DEACTIVATE: self.on_deactivate,
            Transition.TRANSITION_CLEANUP: self.on_cleanup,
            Transition.TRANSITION_ACTIVE_SHUTDOWN: self.on_shutdown,
            Transition.TRANSITION_CALLBACK_ERROR: self.on_error,
            Transition.TRANSITION_ON_ACTIVATE_ERROR: self.on_error,
            Transition.TRANSITION_ON_CLEANUP_ERROR: self.on_error,
            Transition.TRANSITION_ON_CONFIGURE_ERROR: self.on_error,
            Transition.TRANSITION_ON_ERROR_ERROR: self.on_error,
            Transition.TRANSITION_ON_DEACTIVATE_ERROR: self.on_error,
            Transition.TRANSITION_ON_SHUTDOWN_ERROR: self.on_error,
        }
        if request.transition.id in transition_callback:
            response.success = transition_callback[request.transition.id]()
        else:
            response.success = False

        return response

    def on_configure(self):
        self.get_logger().info(f"[{self.get_name()}] Configuring...")
        model_file = self.get_parameter(
            "model_file").get_parameter_value().string_value
        try:
            self.domain_expert = DomainExpert(model_file)
            self.state = State.PRIMARY_STATE_INACTIVE
        except:
            self.get_logger().error("PDDL syntax error")
            return False

        self.get_logger().info(f"[{self.get_name()}] Configured")
        return True

    def on_activate(self):
        self.get_logger().info(f"[{self.get_name()}] Activating...")
        self.state = State.PRIMARY_STATE_ACTIVE
        self.get_logger().info(f"[{self.get_name()}] Activated")
        return True

    def on_deactivate(self):
        self.get_logger().info(f"[{self.get_name()}] Deactivating...")
        self.state = State.PRIMARY_STATE_INACTIVE
        self.get_logger().info(f"[{self.get_name()}] Deactivated")
        return True

    def on_cleanup(self):
        self.get_logger().info(f"[{self.get_name()}] Cleaning up...")
        self.get_logger().info(f"[{self.get_name()}] Cleaned up")
        return True

    def on_shutdown(self):
        self.get_logger().info(f"[{self.get_name()}] Shutting down...")
        self.state = State.PRIMARY_STATE_FINALIZED
        self.get_logger().info(f"[{self.get_name()}] Shutted down")
        return True

    def on_error(self):
        self.get_logger().error(f"[{self.get_name()}] Error transition")
        return True


    def get_domain_service_callback(self, request, response):
        self.get_logger().info(f'get_domain::Incoming request: {request}')

        if self.domain_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            response.domain = self.domain_expert.getDomain()
            response.success = True
        return response

    def get_domain_name_service_callback(self, request, response):
        self.get_logger().info(f'get_domain_name::Incoming request: {request}')

        if self.domain_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            response.name = self.domain_expert.getName()
            response.success = True
        return response

    def get_domain_types_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_types::Incoming request: {request}')

        if self.domain_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            response.types = self.domain_expert.getTypes()
            response.success = True
        return response

    def get_domain_constants_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_constants::Incoming request: {request}')

        if self.domain_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            response.constants = self.domain_expert.getConstants(request.type)
            response.success = True
        return response

    def get_domain_actions_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_actions::Incoming request: {request}')

        if self.domain_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            response.actions = self.domain_expert.getActions()
            response.success = True
        return response

    def get_domain_action_details_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_action_details::Incoming request: {request}')
        if self.domain_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            action = self.domain_expert.getAction(
                request.action, request.parameters)
            if action is None:
                self.get_logger().warn(
                    f'Requesting a non-existing action [{request.action}]')
                response.error_info = "Action not found"
                response.success = False
            else:
                response.action = action
                response.success = True
        return response

    def get_domain_durative_actions_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_durative_actions::Incoming request: {request}')

        if self.domain_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            response.actions = self.domain_expert.getDurativeActions()
            response.success = True
        return response

    def get_domain_durative_action_details_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_durative_action_details::Incoming request: {request}')

        if self.domain_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            durative_action = self.domain_expert.getDurativeAction(
                request.durative_action, request.parameters)
            if durative_action is None:
                self.get_logger().warn(
                    f'Requesting a non-existing durative action [{request.durative_action}]')
                response.error_info = "Durative action not found"
                response.success = False
            else:
                response.durative_action = durative_action
                response.success = True
        return response

    def get_domain_predicates_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_predicates::Incoming request: {request}')

        if self.domain_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            response.states = self.domain_expert.getPredicates()
            response.success = True
        return response

    def get_domain_predicate_details_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_predicate_details::Incoming request: {request}')

        if self.domain_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
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

        if self.domain_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
            response.states = self.domain_expert.getFunctions()
            response.success = True
        return response

    def get_domain_function_details_service_callback(self, request, response):
        self.get_logger().info(
            f'get_domain_function_details::Incoming request: {request}')

        if self.domain_expert is None:
            self.get_logger().error("Requesting service in non-active state")
            response.error_info = "Requesting service in non-active state"
            response.success = False
        else:
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
