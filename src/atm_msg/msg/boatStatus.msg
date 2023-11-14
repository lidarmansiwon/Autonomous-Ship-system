std_msgs/Header header 

# mission mode
float64 mode

# rc
float64 rc_mode

# 노드 주기
float64 navi_hz
float64 guid_hz


# 선박 상태 취득 방식
float64 x
float64 y
float64 psi
float64 u
float64 v
float64 r

# command psi 
float64 cmd_psi

# calc pwm [thr1, thr2, thr3, thr4]
float64[] cmd_pwm

# goal data
GoalInfo goal_data