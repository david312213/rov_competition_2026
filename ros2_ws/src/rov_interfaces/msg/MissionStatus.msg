# 自主任务的结构化可观测状态。
# 距离字段中，estimated_advance_distance_m 只是无 DVL 条件下的时间估算。

builtin_interfaces/Time stamp
bool active
string state
string outcome
string message
uint32 frame_id

bool has_target
string target_label
float32 target_area_ratio
float32 grasp_area_threshold
float32 horizontal_error
float32 vertical_error

bool valid_depth
float32 current_depth_m
float32 target_depth_m

bool valid_scan_progress
float32 scan_progress_deg
float32 estimated_advance_distance_m
uint16 search_cycle

bool gripper_command_accepted

# 群体收集测试的附加状态。旧任务发布者保持默认零值。
bool has_cluster
uint16 visible_scallop_count
uint16 locked_cluster_count
float32 cluster_center_x_ratio
float32 cluster_center_y_ratio
float32 cluster_union_area_ratio
uint8 grasp_attempt_index
uint16 completed_cluster_count
string descent_trigger_mode
