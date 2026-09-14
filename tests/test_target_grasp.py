"""Offline target lock, descent and sequential deposit tests: no ROS/hardware."""
from dataclasses import replace
import pytest
from rov_competition.cluster_collection import ClusterCollectionConfig, ClusterCollectionState as S, DescentTrigger
from rov_competition.target_grasp import TargetGraspMission, choose_target, associate_target
from rov_competition.domain import Detection, BoundingBox, MissionObservation
from rov_competition.gripper_execution import GripperExecution


def box(x=500, bottom=800, confidence=.9, size=120):
    return Detection(2, "scallop", confidence, BoundingBox(x-size/2, bottom-size, x+size/2, bottom))


def obs(frame, detections=None, depth=.5):
    return MissionObservation(frame_id=frame, detections=tuple(detections if detections is not None else (box(), box(650, confidence=.8))),
        frame_width=1000, frame_height=1000, perception_valid=True, depth_valid=True,
        depth_m=depth, attitude_valid=True, yaw_deg=0.)


def mission(no_gripper=False):
    config = ClusterCollectionConfig(descent_trigger=DescentTrigger.TARGET_BOTTOM_LINE)
    m = TargetGraspMission(config, stop_after_approach=no_gripper)
    m.start(obs(0, depth=.2), descent_command=.4, now=0)
    m._enter(S.SCANNING, obs(0), 0)
    return m


def ready(no_gripper=False):
    m = mission(no_gripper)
    for i in range(1, 6):
        m.step(obs(i), i*.1)
    assert m.state == S.APPROACHING
    return m


def complete(m, frame, now):
    action = m.awaiting_gripper_action
    request_id = f"r{frame}"
    m.bind_execution(request_id, now, 5)
    before = m.state
    accepted = m.acknowledge_gripper(action, True, "accepted", obs(frame), now)
    assert m.state == before and accepted.motion.is_neutral()
    assert m.execution_status("other", action, "completed", obs(frame), now) is None
    result = m.execution_status(request_id, action, "completed", obs(frame), now+.2, stamp=now+.2)
    assert m.execution_status(request_id, action, "completed", obs(frame), now+.3) is None
    return result


def test_choose_confidence_and_horizontal_tie():
    assert choose_target((box(600,confidence=.95), box(500, confidence=.9)),1000,.5).box.center()[0] == 600
    assert choose_target((box(600),box(510)),1000,.5).box.center()[0] == 510


def test_track_geometry_not_rising_competitor_confidence():
    previous = box()
    same, other = box(510, confidence=.5), box(650, confidence=.99)
    assert associate_target(previous, (other,same),1000,1000) is same
    assert associate_target(previous, (box(800),),1000,1000) is None
    assert associate_target(previous, (box(495),box(505)),1000,1000) is None


def test_threshold_new_frames_only_and_no_gripper_stops():
    m = ready(True)
    for i in range(6, 10):
        d = m.step(obs(i), i*.1)
        assert d.motion.forward == 0
        for _ in range(10):
            m.step(obs(i), i*.1+.01)
        assert m.state == S.APPROACHING
    d = m.step(obs(10), 1.)
    assert d.state == S.COMPLETE and d.gripper_action is None and d.motion.is_neutral()


@pytest.mark.parametrize('bottom,forward', [(649,.20),(650,.10),(799,.10),(800,0.)])
def test_selected_edge_drives_speed_not_group_center(bottom, forward):
    m=ready()
    m._set_target(box(bottom=bottom),obs(6))
    d=m._step_approaching(obs(6),.6)
    assert d.motion.forward == forward


def test_first_crossing_stops_until_confirmed_even_if_edge_retreats():
    m=ready()
    m._step_approaching(obs(6),.6)
    m._set_target(box(bottom=790),obs(7))
    assert m._step_approaching(obs(7),.7).motion.forward == 0


def descending():
    m=ready()
    for i in range(6,11):
        d=m.step(obs(i),i*.1)
    assert d.gripper_action == 'open'
    complete(m,11,1.1)
    assert m.state == S.GRASP_DESCENDING
    return m


def test_diagonal_and_loss_latch_vertical_only():
    m=descending()
    d=m.step(obs(12),1.4)
    assert d.motion.forward==.1 and d.motion.vertical==-.35
    d=m.step(obs(13,[]),1.5)
    assert d.motion.forward==0 and d.motion.yaw==0 and d.motion.vertical==-.35
    assert m.descent_vertical_only
    d=m.step(obs(14),1.6)
    assert d.motion.forward==0 and d.motion.vertical==-.35


def test_clipping_latches_and_offcenter_preserves_descent():
    m=descending()
    d=m.step(obs(12,[box(585)]),1.4)
    assert d.motion.forward==0 and d.motion.vertical==-.35
    m._set_target(box(585,bottom=990),obs(13))
    d=m.step(obs(13,[box(585,bottom=1000)]),1.5)
    assert m.descent_vertical_only and d.motion.yaw==0


def test_video_pause_is_not_target_loss_and_descent_timeout_aborts():
    m=descending()
    d=m.step(replace(obs(12,[]),perception_valid=False),1.4)
    assert d.motion.is_neutral() and not m.descent_vertical_only
    assert m.step(obs(13),12).state == S.ABORTED


@pytest.mark.parametrize('state',['failed','cancelled'])
def test_servo_failure_never_recovers_or_counts(state):
    m=ready()
    m._action('raise',S.RAISING_CLAW,obs(10),1)
    m.bind_execution('r',1,5)
    d=m.execution_status('r','raise',state,obs(10),1.1)
    assert d.state==S.ABORTED and d.motion.is_neutral() and d.gripper_action is None
    assert m.total_grasp_attempt_count==0


def test_stale_execution_aborts_and_service_acceptance_does_not_advance():
    m=ready()
    m._action('raise',S.RAISING_CLAW,obs(10),1)
    m.bind_execution('r',1,5)
    m.acknowledge_gripper('raise',True,'ok',obs(10),1)
    assert m.step(obs(11),2.01).state==S.ABORTED


def test_complete_during_pause_does_not_advance():
    m=ready()
    m._action('raise',S.RAISING_CLAW,obs(10),1)
    m.bind_execution('r',1,5)
    assert m.execution_status('r','raise','completed',obs(10),1.2,stamp=1.2,allow_transition=False) is None
    assert m.state==S.RAISING_CLAW
    assert m.execution_status('r','raise','completed',obs(11),1.3,stamp=1.3).gripper_action=='open'


def test_three_deposits_before_body_ascent_and_then_leave():
    m=descending()
    now=2.; frame=20
    for attempt in range(3):
        # Simulated pressure contact, preserving the actual mechanical states.
        m.bottom_depth_m=.5
        d=m._action('close',S.CLOSING_GRIPPER,obs(frame),now)
        actions=[]
        for expected in ('close','raise','open'):
            assert d.gripper_action==expected
            actions.append(expected)
            assert d.motion.is_neutral()
            d=complete(m,frame,now)
            frame+=1; now+=.3
        assert m.state==S.WAITING_DROP and m.total_grasp_attempt_count==attempt
        assert m.step(obs(frame),now+.2).motion.is_neutral()
        now+=1.2
        d=m.step(obs(frame+1),now)
        frame+=2
        for expected in ('close','reset'):
            assert d.gripper_action==expected and d.motion.is_neutral()
            actions.append(expected)
            d=complete(m,frame,now)
            frame+=1;now+=.3
        assert actions==['close','raise','open','close','reset']
        assert m.total_grasp_attempt_count==attempt+1 and m.state==S.CLEARING_BOTTOM
        assert m.step(obs(frame,depth=.5),now).motion.vertical>0
        frame+=1;now+=.1
        m.step(obs(frame,depth=.35),now)
        frame+=1;now+=1.1
        d=m.step(obs(frame,depth=.35),now)
        frame+=1;now+=.1
        if attempt<2:
            assert m.state==S.REACQUIRING_GROUP and m.selected_target is None
            for _ in range(3):
                m.step(obs(frame,depth=.35),now)
                frame+=1;now+=.1
            assert m.state==S.ALIGNING and m.selected_target is not None
        else:
            assert m.state==S.LEAVING_CLUSTER and m.completed_cluster_count==1


def test_execution_wait_serialization_deadline_and_cancel():
    e=GripperExecution()
    e.begin('1','commissioning','raise',0,1,1)
    with pytest.raises(ValueError): e.begin('2','commissioning','reset',0,1,1)
    e.poll(.5,True); assert e.state=='running'
    e.poll(1,False); assert e.state=='running'
    e.poll(1.9,False); assert e.state=='running'
    e.poll(2,False); assert e.state=='completed'
    e.begin('2','commissioning','open',3,0,1)
    e.fail('stop',cancelled=True)
    e.poll(100,False); assert e.state=='cancelled'
    e.begin('3','commissioning','reset',101,1,1)
    e.poll(107,True); assert e.state=='failed'


def test_complete_three_cycles_with_simulated_depth_and_delayed_servo():
    m=ready(); depth=.5; now=.5; frame=5; execution=GripperExecution()
    actions=[]; states=[]; request=0
    for tick in range(1000):
        now+=.1; frame+=1
        o=obs(frame,depth=depth)
        decision=None
        if m.execution_request_id:
            execution.poll(now,False)
            decision=m.execution_status(execution.request_id,execution.action,execution.state,o,now,stamp=now)
        if decision is None:
            decision=m.step(o,now)
        states.append(m.state)
        assert m.state != S.ABORTED, decision.message
        if decision.gripper_action:
            request+=1
            actions.append(decision.gripper_action)
            m.bind_execution(str(request),now,5)
            execution.begin(str(request),'commissioning',decision.gripper_action,now,.1,.3)
        if m.state in {S.CLOSING_GRIPPER,S.RAISING_CLAW,S.RELEASING_CATCH,S.WAITING_DROP,S.CLOSING_FOR_RESET,S.RESETTING_CLAW}:
            assert decision.motion.is_neutral()
            assert depth==pytest.approx(.8)
        if decision.motion.vertical<0:
            depth=min(.8,depth+.025)
        if decision.motion.vertical>0:
            assert m.total_grasp_attempt_count>0
            depth=max(.65,depth-.025)
        if m.state==S.LEAVING_CLUSTER:
            break
    assert m.state==S.LEAVING_CLUSTER
    assert actions==['open','close','raise','open','close','reset']*3
    assert m.total_grasp_attempt_count==3
    assert S.REACQUIRING_GROUP in states and S.INTER_GRAB_ADVANCING not in states


def test_reacquisition_never_blind_grabs_when_group_gone():
    m=ready(); m._clear_cluster_lock(); m.group_reference=(.5,.74)
    m._enter(S.REACQUIRING_GROUP,obs(6),.6); m.reacquire_started_at=.6
    for i in range(7,20):
        d=m.step(obs(i,[]),i*.1)
        assert d.gripper_action is None
    assert m.state==S.SCANNING


def test_invalid_depth_during_servo_aborts_without_reset():
    m=ready(); m._action('raise',S.RAISING_CLAW,obs(6),.6); m.bind_execution('r',.6,5)
    d=m.step(replace(obs(7),depth_valid=False),.7)
    assert d.state==S.ABORTED and d.gripper_action is None
