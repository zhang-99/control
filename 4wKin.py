import numpy as np
import math


#四舵轮底盘(4WIS)运动学方程推导
#1---------------4
# -             - 
# -             -
# -             -
# -             -
# -             -
# -             -
#2---------------3




CHASSIS_HALF_WIDTH=0.7

def FourSteeredWheeledRobotInverseKinematics(Vx, Vy, Omega):

    h = CHASSIS_HALF_WIDTH
    hw = h * Omega
    target_steer_theta = [
       math.atan2((Vy + hw), (Vx - hw)),#fl
       math.atan2((Vy - hw), (Vx - hw)),#rl
       math.atan2((Vy - hw), (Vx + hw)),#rr
       math.atan2((Vy + hw), (Vx + hw)),#fr
    ]

    target_drive_vel = [
        math.sqrt(math.pow(Vx - hw, 2) + math.pow(hw + Vy, 2)) ,#/ WHEEL_RADIUS, #fl
        math.sqrt(math.pow(Vx - hw, 2) + math.pow(hw - Vy, 2)) ,#/ WHEEL_RADIUS, #rl
        math.sqrt(math.pow(Vx + hw, 2) + math.pow(hw - Vy, 2)) ,#/ WHEEL_RADIUS, #rr
        math.sqrt(math.pow(Vx + hw, 2) + math.pow(hw + Vy, 2)) ,#/ WHEEL_RADIUS, #fr
    ]

    return target_drive_vel,target_steer_theta



def FourSteeredWheeledRobotForwardKinematics(v1,v2,v3,v4,q1,q2,q3,q4):
    a=CHASSIS_HALF_WIDTH
    b=CHASSIS_HALF_WIDTH
    K=4*(a*a+b*b)
    
    q=1/4
    p_8_3_p=np.matrix([[q,   0,   q,  0,  q,  0,  q,  0],
                      [0,   q,   0,  q,  0,  q,  0,  q],
                      [-b/K,a/K,-b/K,-a/K,b/K,-a/K,b/K,a/K]])
    
    c1=math.cos(q1)
    s1=math.sin(q1)
    
    c2=math.cos(q2)
    s2=math.sin(q2)
    
    c3=math.cos(q3)
    s3=math.sin(q3)
    
    c4=math.cos(q4)
    s4=math.sin(q4)
    x_8_4=np.matrix([[c1,0,0,0],
                    [s1,0,0,0],
                    [0,c2,0,0],
                    [0,s2,0,0],
                    [0,0,c3,0],
                    [0,0,s3,0],
                    [0,0,0,c4],
                    [0,0,0,s4]])

    # print(x_8_4.shape)


    v_s=np.matrix([v1,v2,v3,v4])


    status=p_8_3_p*(x_8_4*v_s.T)


    return status
    


    


print("逆运动学", 0.3,0.1,0.5)
target_drive_vel,target_steer_theta=FourSteeredWheeledRobotInverseKinematics(0.3,0.1,0.5)
print("v",target_drive_vel ,"theta",target_steer_theta)




status=FourSteeredWheeledRobotForwardKinematics(target_drive_vel[0],
                                                target_drive_vel[1],
                                                target_drive_vel[2],
                                                target_drive_vel[3],
                                                target_steer_theta[0],
                                                target_steer_theta[1],
                                                target_steer_theta[2],
                                                target_steer_theta[3],
                                                )

print("正运动学" ,status)






