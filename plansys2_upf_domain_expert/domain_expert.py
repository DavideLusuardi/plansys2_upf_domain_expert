from plansys2_msgs import msg

import unified_planning as up
from unified_planning.io import PDDLReader
from unified_planning.model.action import InstantaneousAction
from unified_planning.model.operators import OperatorKind
from unified_planning.model.effect import EffectKind
from unified_planning.model import timing
from unified_planning.model.parameter import Parameter

from typing import List, Dict


class DomainExpert():

    def __init__(self, domain_filename: str):
        self.domain = PDDLReader().parse_problem(domain_filename, None)
        
        self.domain_pddl = None 
        with open(domain_filename) as f:
            ll = f.readlines()
            self.domain_pddl = ''.join(ll)
        
        # TODO: fix the mapping, there are some UnsupportedConstruct
        self.map_types = {
            OperatorKind.BOOL_CONSTANT  : msg.Node.NUMBER, # TODO
            OperatorKind.INT_CONSTANT   : msg.Node.NUMBER,
            OperatorKind.REAL_CONSTANT  : msg.Node.NUMBER,
            OperatorKind.AND            : msg.Node.AND,
            OperatorKind.OR             : msg.Node.OR,
            OperatorKind.NOT            : msg.Node.NOT,
            # OperatorKind.IMPLIES        : msg.Node.PREDICATE,
            # OperatorKind.IFF            : msg.Node.PREDICATE,
            # OperatorKind.EXISTS         : msg.Node.PREDICATE,
            # OperatorKind.FORALL         : msg.Node.PREDICATE,
            OperatorKind.FLUENT_EXP     : msg.Node.PREDICATE, # TODO: fluent_exp is function or predicate
            # OperatorKind.PARAM_EXP      : msg.Node.EXPRESSION,
            # OperatorKind.VARIABLE_EXP   : msg.Node.EXPRESSION,
            # OperatorKind.OBJECT_EXP     : msg.Node.EXPRESSION,
            # OperatorKind.TIMING_EXP     : msg.Node.EXPRESSION,
            OperatorKind.PLUS           : msg.Node.EXPRESSION,
            OperatorKind.MINUS          : msg.Node.EXPRESSION,
            OperatorKind.TIMES          : msg.Node.EXPRESSION,
            OperatorKind.DIV            : msg.Node.EXPRESSION,
            OperatorKind.EQUALS         : msg.Node.EXPRESSION,
            OperatorKind.LE             : msg.Node.EXPRESSION,
            OperatorKind.LT             : msg.Node.EXPRESSION,

            EffectKind.ASSIGN           : msg.Node.FUNCTION_MODIFIER,
            EffectKind.INCREASE         : msg.Node.FUNCTION_MODIFIER,
            EffectKind.DECREASE         : msg.Node.FUNCTION_MODIFIER,
        }

        self.map_exp_types = {
            OperatorKind.PLUS           : msg.Node.ARITH_ADD,
            OperatorKind.MINUS          : msg.Node.ARITH_SUB,
            OperatorKind.TIMES          : msg.Node.ARITH_MULT,
            OperatorKind.DIV            : msg.Node.ARITH_DIV,
            OperatorKind.EQUALS         : msg.Node.COMP_EQ,
            OperatorKind.LE             : msg.Node.COMP_LE,
            OperatorKind.LT             : msg.Node.COMP_LT,
        }
        
        # TODO: SCALE_UP, SCALE_DOWN not supported
        self.map_fn_modifier_types = {
            EffectKind.ASSIGN           : msg.Node.ASSIGN,
            EffectKind.INCREASE         : msg.Node.INCREASE,
            EffectKind.DECREASE         : msg.Node.DECREASE,
        }

    # TODO
    def getDomain(self):
        # return str(self.domain)
        return self.domain_pddl

    def getName(self) -> str:
        return self.domain.name

    def getTypes(self):
        return list(map(lambda t: t.name, self.domain.user_types))

    # TODO: check if type is valid
    def getConstants(self, type: str):
        # return self.objects(type)
        constants = filter(lambda o: o.type.name == type, self.domain._objects)
        return list(map(lambda c: c.name, constants))

    def getActions(self):
        instantaneous_actions = filter(lambda a: isinstance(a, InstantaneousAction), self.domain.actions)
        # instantaneous_actions = set(self.domain.actions) - set(self.domain.durative_actions)
        return list(map(lambda a: a.name, instantaneous_actions))

    def constructParameters(self, parameters: List[up.model.Parameter], params_map: Dict[str, str]):
        params_msg = list()
        for param in parameters:
            param_msg = msg.Param()
            param_msg.name = param.name if params_map is None else params_map[param.name]
            param_msg.type = param.type.name
            
            sub_types = filter(lambda ut: ut.father == param.type, self.domain.user_types)
            param_msg.sub_types = list(map(lambda ut: ut.name, sub_types))

            params_msg.append(param_msg)
        
        return params_msg

    def constructTree(self, fnode: up.model.fnode.FNode, nodes: List[msg.Node], params_map: Dict[str, str]) -> msg.Node:
        print(f"fnode: {fnode}")
        print(f"node type: {fnode.node_type}")
        print(f"node id: {fnode.node_id}")
        print(f"args: {fnode.args}\n")

        node = msg.Node()
        # node.node_id = fnode.node_id # TODO
        node.node_id = len(nodes)
        node.children = list()
        nodes.append(node)

        node.node_type = self.map_types[fnode.node_type]
        if fnode.node_type in self.map_exp_types:
            node.expression_type = self.map_exp_types[fnode.node_type]

        # elif fnode.node_type in self.map_fn_modifier_types: # TODO: is needed in this function?
        #     node.modifier_type = self.map_fn_modifier_types[fnode.node_type]
        
        elif fnode.node_type in (OperatorKind.INT_CONSTANT, OperatorKind.REAL_CONSTANT):
            node.value = float(fnode.constant_value())

        
        if fnode.node_type == OperatorKind.FLUENT_EXP:
            print(f"fluent type: {fnode.fluent().type}\n")
            # TODO: are there only predicates and functions?
            if fnode.fluent().type.is_bool_type():
                node.node_type = msg.Node.PREDICATE
            else:
                node.node_type = msg.Node.FUNCTION

            node.name = fnode.fluent().name

            parameters = list()
            for child_fnode in fnode.args:
                if child_fnode.is_parameter_exp():
                    parameters.append(child_fnode.parameter())
                elif child_fnode.is_object_exp():
                    parameters.append(child_fnode.object())
                else:
                    raise # TODO
            node.parameters = self.constructParameters(parameters, params_map) # TODO: reuse previously constructed parameters

        else:
            for child_fnode in fnode.args:
                child_node = self.constructTree(child_fnode, nodes, params_map)
                node.children.append(child_node.node_id)

        return node

    def constructEffectsTree(self, effects, nodes, params_map: Dict[str, str]):
        and_node = msg.Node()
        and_node.node_type = self.map_types[OperatorKind.AND]
        and_node.node_id = len(nodes)
        and_node.children = list()
        nodes.append(and_node)

        for effect in effects:
            print("======effect=========")
            print(f"effect: {effect}")
            print(f"effect.fluent: {effect.fluent}")
            print(f"effect.value: {effect.value}")
            print(f"effect.kind: {effect.kind}")
            
            if effect.value.is_bool_constant():
                node_parent = and_node
                if effect.value.bool_constant_value() == False:
                    not_node = msg.Node()
                    not_node.node_type = self.map_types[OperatorKind.NOT]
                    not_node.node_id = len(nodes)
                    not_node.children = list()
                    and_node.children.append(not_node.node_id)
                    nodes.append(not_node)
                    node_parent = not_node

                fluent_node = self.constructTree(effect.fluent, nodes, params_map)
                node_parent.children.append(fluent_node.node_id)

            else:
                node = msg.Node()
                node.node_id = len(nodes)
                and_node.children.append(node.node_id)
                nodes.append(node)

                node.node_type = self.map_types[effect.kind] # TODO: map_types can be modified removing EffectKind
                node.modifier_type = self.map_fn_modifier_types[effect.kind]
                fluent_node = self.constructTree(effect.fluent, nodes, params_map)
                value_node = self.constructTree(effect.value, nodes, params_map)
                node.children = [fluent_node.node_id, value_node.node_id]

    def getAction(self, action: str, parameters: List[str]):
        instantaneous_actions = filter(lambda a: isinstance(a, InstantaneousAction), self.domain.actions)
        for action in instantaneous_actions:
            if action.name == action:
                action_msg = msg.Action()
                action_msg.name = action.name
                
                params_map = dict([(p.name, parameters[i] if i<len(parameters) else f'?{i}') for i,p in enumerate(action.parameters)])
                action_msg.parameters = self.constructParameters(action.parameters, params_map)
                
                action_msg.preconditions = msg.Tree()
                action_msg.preconditions.nodes = list()
                for precondition in action.preconditions: # TODO: check if correct, can we have many preconditions?
                    print("======precondition=========")
                    self.constructTree(precondition, action_msg.preconditions.nodes, params_map)

                action_msg.effects = msg.Tree()
                action_msg.effects.nodes = list()
                self.constructEffectsTree(action.effects, action_msg.effects.nodes, params_map)
                
                return action_msg

        return None

    def getDurativeActions(self):
        return list(map(lambda a: a.name, self.domain.durative_actions))

    def getDurativeAction(self, action: str, parameters: List[str]):
        for durative_action in self.domain.durative_actions:
            if durative_action.name == action:
                durative_action_msg = msg.DurativeAction()
                durative_action_msg.name = durative_action.name
                
                params_map = dict([(p.name, parameters[i] if i<len(parameters) else f'?{i}') for i,p in enumerate(durative_action.parameters)])
                durative_action_msg.parameters = self.constructParameters(durative_action.parameters, params_map)

                # TODO: add and-node at top of the tree
                for time_interval, conditions in durative_action.conditions.items():
                    if time_interval.lower == timing.StartTiming() and time_interval.upper == timing.EndTiming():
                        for condition in conditions:
                            self.constructTree(condition, durative_action_msg.over_all_requirements.nodes, params_map)

                    elif time_interval.lower == timing.StartTiming():
                        for condition in conditions:
                            self.constructTree(condition, durative_action_msg.at_start_requirements.nodes, params_map)

                    elif time_interval.upper == timing.EndTiming():
                        for condition in conditions:
                            self.constructTree(condition, durative_action_msg.at_end_requirements.nodes, params_map)
                
                for time, effects in durative_action.effects.items():
                    if time == timing.StartTiming():
                        self.constructEffectsTree(effects, durative_action_msg.at_start_effects.nodes, params_map)
                    else:
                        self.constructEffectsTree(effects, durative_action_msg.at_end_effects.nodes, params_map)

                return durative_action_msg

        return None

    def getPredicates(self):
        predicates = filter(lambda f: f.type.is_bool_type(), self.domain.fluents)
        states = list()
        for p in predicates:
            pred = msg.Node()
            pred.node_type = msg.Node.PREDICATE
            pred.name = p.name
            states.append(pred)
        return states

    def getPredicate(self, predicate: str):
        predicates = filter(lambda f: f.type.is_bool_type(), self.domain.fluents)
        for predicate in predicates:
            if predicate.name == predicate:
                predicate_msg = msg.Node()
                predicate_msg.node_type = msg.Node.PREDICATE # TODO: plansys2 sets UNKNOWN
                predicate_msg.name = predicate.name
                # predicate_msg.parameters = list()
                # for i, param in enumerate(predicate.signature):
                #     param_msg = msg.Param()
                #     # param_msg.name = param.name
                #     param_msg.name = f"?{param.type}{i}"
                #     param_msg.type = param.type.name
                    
                #     sub_types = filter(lambda ut: ut.father == param.type, self.domain.user_types)
                #     param_msg.sub_types = list(map(lambda ut: ut.name, sub_types))

                #     predicate_msg.parameters.append(param_msg)

                params_map = dict([(p.name, f"?{p.type}{i}") for i,p in enumerate(predicate.signature)])
                predicate_msg.parameters = self.constructParameters(predicate.signature, params_map)

                return predicate_msg

        return None


    def getFunctions(self):
        functions = filter(lambda f: f.type.is_real_type(), self.domain.fluents)
        states = list()
        for f in functions:
            func = msg.Node()
            func.node_type = msg.Node.FUNCTION
            func.name = f.name
            states.append(func)
        return states

    def getFunction(self, function: str):
        functions = filter(lambda f: f.type.is_real_type(), self.domain.fluents)
        for function in functions:
            if function.name == function:
                function_msg = msg.Node()
                function_msg.node_type = msg.Node.FUNCTION
                function_msg.name = function.name
                # function_msg.parameters = list()
                # for i, param in enumerate(function.signature):
                #     param_msg = msg.Param()
                #     # param_msg.name = param.name
                #     param_msg.name = function"?{param.type}{i}"
                #     param_msg.type = param.type.name
                    
                #     sub_types = filter(lambda ut: ut.father == param.type, self.domain.user_types)
                #     param_msg.sub_types = list(map(lambda ut: ut.name, sub_types))

                #     function_msg.parameters.append(param_msg)
                
                params_map = dict([(p.name, f"?{p.type}{i}") for i,p in enumerate(function.signature)])
                function_msg.parameters = self.constructParameters(function.signature, params_map)

                return function_msg

        return None

