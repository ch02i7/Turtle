uint16[] motor_index  # 要控制的电机索引数组（0-11）
uint16[] modes        # 控制模式数组
float32[] q_targets   # 目标位置数组（rad）
float32[] dq_targets  # 目标速度数组（rad/s）
float32[] tau_targets # 目标力矩数组（Nm）
float32[] kp_gains    # 刚度系数数组
float32[] kd_gains    # 阻尼系数数组

float32 lx
float32 ly
float32 rx
float32 ry
uint32 user_cmd