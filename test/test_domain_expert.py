from plansys2_msgs import msg
from plansys2_upf_domain_expert import DomainExpert
from ament_index_python.packages import get_package_share_directory

import pytest


package_share_directory = get_package_share_directory('plansys2_upf_domain_expert')
domain_pddl_file = f'{package_share_directory}/test/pddl/domain1.pddl'
domain2_pddl_file = f'{package_share_directory}/test/pddl/domain2.pddl'

@pytest.fixture
def domain() -> DomainExpert:
    domain = DomainExpert(domain_pddl_file)
    return domain


def test_getDomain(domain: DomainExpert):
    assert(domain.getDomain())

def test_getName(domain: DomainExpert):
    assert(domain.getName()=='simple')

def test_getTypes(domain: DomainExpert):
    tt = sorted(domain.getTypes())
    assert(tt==['robot','room'])

def test_getConstants(domain: DomainExpert):
    cc = sorted(domain.getConstants('room'))
    assert(cc==['livingroom'])

    cc = sorted(domain.getConstants('robot'))
    assert(len(cc)==0)

def test_getActions(domain: DomainExpert):
    assert(domain.getActions()==['test_action'])

def test_getAction(domain: DomainExpert):
    action = domain.getAction('test_action', [])
    assert(action is not None)
    assert(action.name == 'test_action')

def test_getDurativeActions(domain: DomainExpert):
    aa = sorted(domain.getDurativeActions())
    assert(aa==sorted(['move', 'askcharge', 'charge']))

def test_getDurativeAction(domain: DomainExpert):
    action = domain.getDurativeAction('move', [])
    assert(action is not None)
    assert(action.name == 'move')

def test_getPredicates(domain: DomainExpert):
    pp = sorted(map(lambda p: p.name, domain.getPredicates()))
    assert(pp==sorted(['robot_at','connected','battery_full','battery_low','charging_point_at']))

def test_getPredicate(domain: DomainExpert):
    assert(domain.getPredicate('robot_at') is not None)

def test_getFunctions(domain: DomainExpert):
    ff = sorted(map(lambda f: f.name, domain.getFunctions()))
    assert(ff==sorted(['distance_travelled']))

def test_getFunction(domain: DomainExpert):
    assert(domain.getFunction('distance_travelled') is not None)

