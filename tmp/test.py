import unified_planning as up
from unified_planning.io import PDDLReader, PDDLWriter
from unified_planning.model.action import InstantaneousAction
from unified_planning.model.operators import OperatorKind
# from plansys2_msgs import msg


reader = PDDLReader()
domain = reader.parse_problem(
    '/home/davide/plansys2_ws/src/plansys2_upf_domain_expert/tmp/domain.pddl', None)

# print(domain.name)
# print(domain.user_types[0].name)
# print(domain._objects)
# print(list(domain.objects(domain.user_types[0])))
# print(domain.fluents)
# print(domain.fluent('test_predicate'))
# print(list(filter(lambda f: f.type.is_bool_type(), domain._fluents)))
# print(list(filter(lambda f: f.type.is_real_type(), domain._fluents)))
# print(type(list(domain.actions)[0]))
# print(list(domain.durative_actions))


# action = 'test_action1'
# instantaneous_actions = filter(lambda a: isinstance(a, InstantaneousAction), domain.actions)
# for a in instantaneous_actions:
#     if a.name == action:
#         print(a.effects)
#         for effect in a.effects:
#             print(effect)
#             print(effect.fluent)
#             print(effect.value)
#             print(effect.condition)
#             print(effect.kind)


# print(set(domain.actions))
# print(set(domain.durative_actions))
# print(set(domain.actions) - set(domain.durative_actions))
# for a in domain.durative_actions:
#     print(a.name)
#     print(a.parameters)
#     print(a.duration)
#     print(a.conditions)
#     for k,v in a.conditions.items():
#         print(type(k))
#         print(type(v[0]))
#         print(v[0].node_type)
#     print(a.effects)


instantaneous_actions = list(filter(lambda a: isinstance(a, InstantaneousAction), domain.actions))
action = instantaneous_actions[0]
p = list(action.parameters)[0]
# print(p.type)
# print(p.type.father)
for type in domain.user_types:
    # print(type.father)
    if type.father == p.type:
        print(type)
