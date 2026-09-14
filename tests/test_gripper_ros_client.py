"""Built-interface offline test: never initializes ROS or connects MAVLink."""
from types import SimpleNamespace as NS
import pytest

rclpy=pytest.importorskip('rclpy')
pytest.importorskip('rov_interfaces.srv')
from rov_interfaces.srv import SetGripper
from rov_interfaces.msg import GripperExecutionStatus
from rov_competition.cluster_collection_runtime import ClusterCollectionNode, DatasetDriveError
from rov_competition.cluster_collection import ClusterCollectionState as S
from test_target_grasp import ready, obs


def message(request_id='r',action='raise',state='completed',stamp=1.2):
    return NS(request_id=request_id,source='commissioning',action=action,state=state,
              stamp=NS(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9)))


def test_built_interface_constants_and_request_identity():
    request=SetGripper.Request()
    assert (request.OPEN,request.CLOSE,request.RAISE,request.RESET)==(1,2,3,4)
    request.request_id='unique'
    response=SetGripper.Response();response.request_id=request.request_id
    status=GripperExecutionStatus();status.request_id=request.request_id
    assert status.request_id==response.request_id=='unique'


def test_delayed_acceptance_and_correlated_completion():
    m=ready();m._action('raise',S.RAISING_CLAW,obs(10),1);m.bind_execution('r',1,5)
    done=[False]
    future=NS(done=lambda:done[0],result=lambda:NS(request_id='r',success=True))
    node=NS(execution_future=future,execution_feedback=(message(),1.2))
    assert ClusterCollectionNode.poll_execution(node,m,obs(11),1.2,allow_transition=True) is None
    assert m.state==S.RAISING_CLAW
    done[0]=True
    node.execution_feedback=(message(stamp=1.3),1.3)
    d=ClusterCollectionNode.poll_execution(node,m,obs(12),1.3,allow_transition=True)
    assert d.gripper_action=='open'


def test_rejected_acceptance_and_missing_status_abort():
    m=ready();m._action('raise',S.RAISING_CLAW,obs(10),1);m.bind_execution('r',1,5)
    node=NS(execution_future=NS(done=lambda:True,result=lambda:NS(request_id='other',success=True)),execution_feedback=None)
    with pytest.raises(DatasetDriveError):
        ClusterCollectionNode.poll_execution(node,m,obs(11),1.2,allow_transition=True)
    node.execution_future=NS(done=lambda:False)
    d=ClusterCollectionNode.poll_execution(node,m,obs(12),2.1,allow_transition=False)
    assert d.state==S.ABORTED


def test_wrong_request_and_old_feedback_never_refresh():
    m=ready();m._action('raise',S.RAISING_CLAW,obs(10),1);m.bind_execution('r',1,5)
    node=NS(execution_future=NS(done=lambda:True,result=lambda:NS(request_id='r',success=True)),
            execution_feedback=(message(request_id='wrong'),1.2))
    assert ClusterCollectionNode.poll_execution(node,m,obs(11),1.2,allow_transition=True) is None
    d=ClusterCollectionNode.poll_execution(node,m,obs(12),2.1,allow_transition=True)
    assert d.state==S.ABORTED
