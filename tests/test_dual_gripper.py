"""Synthetic channels/PWMs below are fixtures, NOT hardware calibration."""
from dataclasses import replace
from pathlib import Path
import json
import csv
import pytest
import yaml
from rov_competition.config import GripperConfig, GripperOutputConfig, GripperSweepConfig, ConfigurationError, load_robot_config
from rov_competition.dual_gripper_calibration import signature, single_step
from rov_competition.domain import GripperAction
from rov_competition.vehicle import VehicleError, MAV_CMD_DO_SET_SERVO
from rov_competition.gripper_test import write_candidate_config, _write_results
from rov_competition.field_setup import validate_gripper_result, activate_gripper, verify_gripper_activation, FieldSetupError
from test_vehicle import make_vehicle, FakeMessage

PACKAGE=Path(__file__).resolve().parents[1]/'ros2_ws/src/rov_competition'


def dual():
    sweep=GripperSweepConfig(1500,1525,25)
    back=GripperSweepConfig(1525,1500,-25)
    return GripperConfig('dual_dof',.125,False,False,
        (GripperOutputConfig(12,sweep,back),),GripperOutputConfig(11,sweep,back),
        {a:.3 for a in ('open','close','raise','reset')},1400,1600)


def test_four_actions_are_independent_and_old_actions_unchanged():
    g=dual()
    for action,channel in [('open',12),('close',12),('raise',11),('reset',11)]:
        assert {ch for step in g.steps_for(action) for ch,pwm in step}=={channel}
    assert g.output_channels==(12,11)


@pytest.mark.parametrize('bad',['same_channel','motor','no_lift','wait_missing','wait_nan','range','no_jaw'])
def test_invalid_dual_config_rejected(bad):
    g=dual()
    with pytest.raises(ConfigurationError):
        if bad=='same_channel': replace(g,lift=replace(g.lift,output_channel=12))
        if bad=='motor': replace(g,lift=replace(g.lift,output_channel=5))
        if bad=='no_lift': replace(g,lift=None)
        if bad=='wait_missing': replace(g,action_wait_s=None)
        if bad=='wait_nan': replace(g,action_wait_s={a:float('nan') for a in g.action_wait_s})
        if bad=='range': replace(g,pwm_max=1510)
        if bad=='no_jaw': replace(g,outputs=())


def test_uncalibrated_blocked_and_manual_step_bounded():
    v,master=make_vehicle(live=True); v._telemetry.armed=True
    with pytest.raises(ConfigurationError,match='标定'):
        replace(v.config,allow_gripper_actuation=True,gripper=dual())
    v.config=replace(v.config,gripper=dual())
    with pytest.raises(VehicleError): v.set_gripper(GripperAction.RAISE)
    assert not master.mav.commands
    changed=single_step(v.config,'raise',1510,1500)
    assert changed.gripper.steps_for('raise')==(((11,1510),),)
    for pwm,previous in [(1700,1690),(1540,1500),(1510,None)]:
        with pytest.raises(ConfigurationError): single_step(v.config,'raise',pwm,previous)


@pytest.mark.parametrize('failure',['reject','timeout','cancel'])
def test_nonblocking_servo_failure_stops_remaining_steps(failure):
    v,master=make_vehicle(live=True); v._telemetry.armed=True
    v.config=replace(v.config,allow_gripper_actuation=True,gripper=replace(dual(),calibrated=True))
    v.set_gripper(GripperAction.RAISE,now=10)
    assert len(master.mav.commands)==1 and v.gripper_active
    assert not v.update_gripper(now=10.5)  # no ACK, no second output
    if failure=='reject':
        v._update_telemetry(FakeMessage('COMMAND_ACK',command=MAV_CMD_DO_SET_SERVO,result=2),10.6)
        with pytest.raises(VehicleError,match='拒绝'): v.update_gripper(now=10.7)
    elif failure=='timeout':
        with pytest.raises(VehicleError,match='超时'): v.update_gripper(now=12.1)
    else:
        v.cancel_gripper(); assert not v.update_gripper(now=10.7)
    assert not v.gripper_active and len(master.mav.commands)==1


def test_each_servo_step_requires_ack_and_final_ack_precedes_completion():
    v,master=make_vehicle(live=True); v._telemetry.armed=True
    v.config=replace(v.config,allow_gripper_actuation=True,gripper=replace(dual(),calibrated=True))
    v.set_gripper(GripperAction.RAISE,now=10)
    ack=FakeMessage('COMMAND_ACK',command=MAV_CMD_DO_SET_SERVO,result=0)
    v._update_telemetry(ack,10.1)
    assert v.update_gripper(now=10.2)
    assert v.gripper_active and len(master.mav.commands)==2
    v._update_telemetry(ack,10.3)
    v.update_gripper(now=10.4)
    assert not v.gripper_active
    assert all(command[4]==11 for command in master.mav.commands)


def profile_data():
    def output(channel):
        return dict(output_channel=channel,open=dict(start_pwm=1500,end_pwm=1525,step_pwm=25),
                    close=dict(start_pwm=1525,end_pwm=1500,step_pwm=-25))
    lift=output(11);lift['raise']=lift.pop('open');lift['reset']=lift.pop('close')
    return dict(calibrated=False,allow_extended_pwm=False,step_interval_s=.125,
        pwm_min=1400,pwm_max=1600,action_wait_s={a:.3 for a in ('open','close','raise','reset')},
        outputs=[output(12)],lift=lift)


def evidence(tmp_path):
    path=tmp_path/'config/robot.yaml'; path.parent.mkdir()
    content=yaml.safe_load((PACKAGE/'config/robot.example.yaml').read_text(encoding='utf-8'))
    content['control']['expected_frame_config']=2
    content['safety']['allow_live_actuation']=True
    content['safety']['allow_ros_arming']=True
    content['gripper']['profiles']['dual_dof']=profile_data()
    path.write_text(yaml.safe_dump(content),encoding='utf-8')
    directory=tmp_path/'output/gripper_tests/fixture'; directory.mkdir(parents=True)
    config=write_candidate_config(path,'dual_dof',directory/'resolved_gripper_test.yaml')
    records=[dict(action=a,step=i,output_channel=ch,pwm=p,ack_result=0)
             for a in ('open','close','raise','reset') for i,step in enumerate(config.gripper.steps_for(a),1) for ch,p in step]
    _write_results(directory,profile='dual_dof',records=records,actual_opened=True,actual_closed=True,
        notes='SIMULATED ONLY',outcome='passed',detail='fixture',extra=dict(actual_raised=True,actual_reset=True,
            calibration_signature=signature(config.gripper),single_step_pwm=None))
    return directory/'result.json',path


def test_calibration_evidence_activation_and_parameter_tampering(tmp_path):
    result,path=evidence(tmp_path)
    validate_gripper_result(tmp_path,result)
    activate_gripper(tmp_path,result,profile='dual_dof',confirmation='ACTIVATE DUAL DOF GRIPPER')
    assert verify_gripper_activation(tmp_path)[0]
    content=yaml.safe_load(path.read_text(encoding='utf-8'))
    content['gripper']['profiles']['dual_dof']['action_wait_s']['raise']=.6
    path.write_text(yaml.safe_dump(content),encoding='utf-8')
    assert not verify_gripper_activation(tmp_path)[0]


@pytest.mark.parametrize('change',['not_raised','single_step','alter_wait','alter_pwm'])
def test_incomplete_or_changed_evidence_cannot_activate(tmp_path,change):
    result,path=evidence(tmp_path)
    data=json.loads(result.read_text(encoding='utf-8'))
    if change=='not_raised': data['actual_raised']=False
    if change=='single_step': data['single_step_pwm']=1510
    if change=='alter_wait': data['calibration_signature']='wrong'
    if change=='alter_pwm':
        commands=result.parent/'commands.csv'
        commands.write_text(commands.read_text(encoding='utf-8').replace('1525','1530'),encoding='utf-8')
    result.write_text(json.dumps(data),encoding='utf-8')
    with pytest.raises(FieldSetupError): validate_gripper_result(tmp_path,result)
