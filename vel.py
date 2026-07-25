import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.animation as animation
from scipy.ndimage import gaussian_filter1d

# ==================== 1. 贝塞尔基础函数（通用，支持任意阶） ====================
def bezier_point(control_points, u):
    """计算贝塞尔曲线上参数u对应的点（德卡斯特里奥算法）"""
    pts = np.array(control_points, dtype=float)
    while len(pts) > 1:
        pts = (1 - u) * pts[:-1] + u * pts[1:]
    return pts[0]

def bezier_derivative(control_points, u, order=1):
    """计算贝塞尔曲线的导数（1阶或2阶）"""
    pts = np.array(control_points, dtype=float)
    n = len(pts) - 1
    
    if order == 1:
        new_pts = pts[1:] - pts[:-1]
        factor = n
    else:  # order == 2
        new_pts = pts[2:] - 2 * pts[1:-1] + pts[:-2]
        factor = n * (n - 1)
    
    pts_d = np.array(new_pts, dtype=float)
    while len(pts_d) > 1:
        pts_d = (1 - u) * pts_d[:-1] + u * pts_d[1:]
    return factor * pts_d[0]

def compute_curvature(control_points, u):
    """计算带符号曲率（正=左转，负=右转）"""
    d1 = bezier_derivative(control_points, u, 1)
    d2 = bezier_derivative(control_points, u, 2)
    norm_d1 = np.linalg.norm(d1)
    if norm_d1 < 1e-12:
        return 0.0
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    return cross / (norm_d1 ** 3)

def compute_arc_length_table(control_points, num_samples=2000):
    """生成弧长查找表: u -> 累计弧长 s"""
    u_array = np.linspace(0, 1, num_samples)
    points = [bezier_point(control_points, u) for u in u_array]
    ds = np.zeros(num_samples)
    for i in range(1, num_samples):
        ds[i] = np.linalg.norm(points[i] - points[i-1])
    s_array = np.cumsum(ds)
    return u_array, s_array

def find_u_by_s(s_target, u_array, s_array):
    """根据弧长s反查参数u（二分查找+线性插值）"""
    if s_target <= 0:
        return 0.0
    if s_target >= s_array[-1]:
        return 1.0
    idx = np.searchsorted(s_array, s_target)
    idx = np.clip(idx, 1, len(s_array) - 1)
    u1, u2 = u_array[idx-1], u_array[idx]
    s1, s2 = s_array[idx-1], s_array[idx]
    return u1 + (u2 - u1) * (s_target - s1) / (s2 - s1 + 1e-12)

def compute_max_curvature(control_points, num_samples=2000):
    """计算整条贝塞尔曲线的最大曲率（绝对值）"""
    max_kappa = 0.0
    for u in np.linspace(0, 1, num_samples):
        kappa = compute_curvature(control_points, u)
        if abs(kappa) > max_kappa:
            max_kappa = abs(kappa)
    return max_kappa

def compute_path_points(control_points, num_points=500):
    """生成路径上的离散点用于绘制"""
    u_values = np.linspace(0, 1, num_points)
    points = [bezier_point(control_points, u) for u in u_values]
    return np.array(points)

def compute_curvature_profile(control_points, num_samples=2000):
    """计算整条路径的曲率曲线（用于平滑和约束）"""
    u_array = np.linspace(0, 1, num_samples)
    curvatures = np.array([compute_curvature(control_points, u) for u in u_array])
    return u_array, curvatures

# ==================== 2. S型速度规划（含角速度约束） ====================
def s_curve_planning_with_constraints(
    total_length, 
    v_start, v_end, 
    v_max, a_max, j_max,
    curvature_profile,  # (u_array, curvatures)
    u_array_arc, s_array_arc,
    omega_max,  # 最大允许角速度 (rad/s)
    dt=0.02
):
    """
    带角速度约束的S型速度规划
    核心：根据曲率计算每个位置的角速度上限，反推线速度上限
    """
    # ---- 2.1 计算角速度约束下的线速度上限 ----
    u_arr, curvatures = curvature_profile
    v_omega_limit = np.zeros_like(curvatures)
    
    for i, kappa in enumerate(curvatures):
        if abs(kappa) > 1e-6:
            # 角速度约束: omega = v * kappa <= omega_max
            # 所以 v <= omega_max / |kappa|
            v_omega_limit[i] = omega_max / abs(kappa)
        else:
            v_omega_limit[i] = float('inf')
    
    # 将角速度约束映射到弧长坐标
    # 先建立 u -> s 的映射
    s_curve = np.linspace(0, total_length, len(curvatures))
    v_omega_limit_s = np.interp(
        s_curve, 
        s_array_arc,  # 注意：s_array_arc是累计弧长，需要匹配
        v_omega_limit
    )
    
    # ---- 2.2 综合速度上限 ----
    # 取：额定速度、曲率限速、角速度限速 的最小值
    v_limit_combined = np.minimum(v_max, v_omega_limit_s)
    
    # 全局Vmax取整个路径的最小值（保守策略）
    v_max_global = np.min(v_limit_combined)
    print(f"角速度约束下的全局Vmax: {v_max_global:.3f} m/s")
    
    # ---- 2.3 标准S型规划 ----
    T_j = a_max / j_max
    
    # 加速段
    v_diff_acc = v_max_global - v_start
    if v_diff_acc > 0:
        T_a = v_diff_acc / a_max
        if T_a >= T_j:
            T1 = T3 = T_j
            T2 = T_a - T_j
            S_acc = v_start * (T1 + T2 + T3) + 0.5 * a_max * (T1**2 + T2**2 + T3**2) + a_max * T1 * T2
        else:
            T1 = T3 = np.sqrt(v_diff_acc / j_max)
            T2 = 0
            S_acc = v_start * (T1 + T3) + 0.5 * j_max * (T1**3 + T3**3)
    else:
        T1 = T2 = T3 = 0
        S_acc = 0
    
    # 减速段
    v_diff_dec = v_max_global - v_end
    if v_diff_dec > 0:
        T_d = v_diff_dec / a_max
        if T_d >= T_j:
            T5 = T7 = T_j
            T6 = T_d - T_j
            S_dec = v_max * (T5 + T6 + T7) - 0.5 * a_max * (T5**2 + T6**2 + T7**2) - a_max * T5 * T6
        else:
            T5 = T7 = np.sqrt(v_diff_dec / j_max)
            T6 = 0
            S_dec = v_max * (T5 + T7) - 0.5 * j_max * (T5**3 + T7**3)
    else:
        T5 = T6 = T7 = 0
        S_dec = 0
    
    T4 = max(0, (total_length - S_acc - S_dec) / v_max_global) if v_max_global > 0 else 0
    
    times, vels, disps = [], [], []
    t1, t2, t3 = T1, T1+T2, T1+T2+T3
    t4, t5, t6 = t3+T4, t3+T4+T5, t3+T4+T5+T6
    t7 = t6 + T7
    
    t = 0
    while t <= t7 + 1e-9:
        if t < t1:
            v = v_start + 0.5 * j_max * t**2
            s = v_start * t + (1/6) * j_max * t**3
        elif t < t2:
            dt = t - t1
            v = v_start + 0.5*j_max*T1**2 + a_max*dt
            s = S_acc - (v_max*T3 - (1/6)*j_max*T3**3) + (v_max - 0.5*j_max*T3**2)*dt + 0.5*a_max*dt**2
        elif t < t3:
            dt = t - t2
            v = v_max - 0.5 * j_max * (T3 - dt)**2
            s = S_acc - (v_max*(T3-dt) - (1/6)*j_max*(T3-dt)**3)
        elif t < t4:
            v = v_max
            s = S_acc + v_max * (t - t3)
        elif t < t5:
            dt = t - t4
            v = v_max - 0.5 * j_max * dt**2
            s = S_acc + (total_length - S_acc - S_dec) + v_max*dt - (1/6)*j_max*dt**3
        elif t < t6:
            dt = t - t5
            v = v_max - 0.5*j_max*T5**2 - a_max*dt
            s = total_length - S_dec + (v_max*T5 - (1/6)*j_max*T5**3) + (v_max - 0.5*j_max*T5**2)*dt - 0.5*a_max*dt**2
        else:
            dt = t - t6
            v = v_end + 0.5 * j_max * (T7 - dt)**2
            s = total_length - (v_end*(T7-dt) - (1/6)*j_max*(T7-dt)**3)
        times.append(t)
        vels.append(v)
        disps.append(min(s, total_length))
        t += dt
    
    return np.array(times), np.array(vels), np.array(disps), t7, v_max_global

# ==================== 3. 主程序 ====================
def main():
    # ---- 3.1 定义五次贝塞尔路径（曲率连续） ----
    control_points = [
        (0.0, 0.0),   # P0 - 起点
        (0.3, 1.8),   # P1
        (1.0, 2.8),   # P2
        (2.0, 2.8),   # P3
        (2.7, 1.8),   # P4
        (3.0, 0.0)    # P5 - 终点
    ]
    
    # ---- 3.2 计算弧长 ----
    u_array, s_array = compute_arc_length_table(control_points, num_samples=2000)
    total_length = s_array[-1]
    path_points = compute_path_points(control_points, 500)
    print(f"路径总弧长: {total_length:.3f} m")
    
    # ---- 3.3 计算曲率曲线（用于角速度约束） ----
    u_curve, curvatures_raw = compute_curvature_profile(control_points, num_samples=2000)
    
    # 可选：对曲率进行平滑滤波（进一步消除微小波动）
    # curvatures_smooth = gaussian_filter1d(curvatures_raw, sigma=2)
    # 这里我们直接用原始曲率（五次贝塞尔曲率已经连续）
    curvatures = curvatures_raw
    
    kappa_max = np.max(np.abs(curvatures))
    print(f"最大曲率: {kappa_max:.3f} 1/m")
    
    # ---- 3.4 AGV物理参数 ----
    g = 9.81
    wheel_base = 0.4      # 轮距 (m)
    cog_height = 0.3      # 重心高度 (m)
    mu = 0.7              # 地面摩擦系数
    omega_max = 2.0       # 电机最大角速度 (rad/s)
    
    # 侧翻/侧滑约束
    a_rollover_max = g * wheel_base / (2 * cog_height)
    a_slip_max = mu * g
    a_lateral_max = min(a_rollover_max, a_slip_max) * 0.8
    
    # 曲率限速
    v_curve_limit = np.sqrt(a_lateral_max / kappa_max) if kappa_max > 1e-6 else float('inf')
    
    # 额定速度
    v_rated = 1.0
    
    # 综合速度上限（先不考虑角速度约束）
    v_max_prelim = min(v_rated, v_curve_limit)
    print(f"曲率限速: {v_curve_limit:.2f} m/s")
    print(f"初步Vmax: {v_max_prelim:.2f} m/s")
    
    # ---- 3.5 S型速度规划（含角速度约束） ----
    v_start, v_end = 0.5, 0.0
    a_max, j_max = 0.4, 0.8
    dt = 0.02
    
    # 创建曲率曲线（用于角速度约束）
    curvature_profile = (u_curve, curvatures)
    
    times, velocities, displacements, total_time, v_max_final = s_curve_planning_with_constraints(
        total_length, v_start, v_end, v_max_prelim, a_max, j_max,
        curvature_profile, u_array, s_array,
        omega_max, dt
    )
    print(f"最终Vmax: {v_max_final:.2f} m/s")
    print(f"总运行时间: {total_time:.2f} s")
    
    # ---- 3.6 计算路径位姿和角速度 ----
    positions = []
    headings = []
    curvatures_at_points = []
    angular_vels = []
    
    for i, s in enumerate(displacements):
        u = find_u_by_s(s, u_array, s_array)
        pos = bezier_point(control_points, u)
        d1 = bezier_derivative(control_points, u, 1)
        heading = np.arctan2(d1[1], d1[0])
        kappa = compute_curvature(control_points, u)
        
        positions.append(pos)
        headings.append(heading)
        curvatures_at_points.append(kappa)
        
        # 角速度计算
        omega = velocities[i] * kappa
        # 限幅
        omega = np.clip(omega, -omega_max, omega_max)
        angular_vels.append(omega)
    
    positions = np.array(positions)
    headings = np.array(headings)
    curvatures_at_points = np.array(curvatures_at_points)
    angular_vels = np.array(angular_vels)
    
    # ---- 3.7 检查角速度连续性 ----
    if len(angular_vels) > 1:
        omega_diff = np.diff(angular_vels)
        max_omega_jump = np.max(np.abs(omega_diff))
        print(f"角速度最大跳变: {max_omega_jump:.4f} rad/s")
        if max_omega_jump > 0.05:
            print("⚠️ 角速度存在明显跳变！")
        else:
            print("✅ 角速度连续平滑")
    
    # ---- 3.8 创建动画 ----
    fig = plt.figure(figsize=(16, 9))
    
    # 主图：路径 + AGV
    ax1 = plt.subplot2grid((2, 4), (0, 0), colspan=2, rowspan=2)
    ax1.plot(path_points[:, 0], path_points[:, 1], 'b-', lw=2, alpha=0.6, label='Path (5th Bezier)')
    ax1.plot([p[0] for p in control_points], [p[1] for p in control_points], 
             'ro--', alpha=0.3, markersize=6, label='Control Points')
    
    # 已走轨迹
    trail_line, = ax1.plot([], [], 'g-', lw=3, alpha=0.8, label='Trail')
    
    # AGV小车
    vehicle_width = 0.3
    vehicle_length = 0.5
    vehicle = Rectangle((0, 0), vehicle_length, vehicle_width, 
                        facecolor='red', edgecolor='darkred', lw=2)
    ax1.add_patch(vehicle)
    
    # 路径点散点（速度颜色映射）
    pos_scatter = ax1.scatter([], [], c=[], cmap='plasma', s=20, alpha=0.6)
    
    # 信息文本
    info_text = ax1.text(0.02, 0.98, '', transform=ax1.transAxes, 
                         fontsize=10, verticalalignment='top',
                         bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)')
    ax1.set_title('5th-order Bezier Path with Angular Velocity Constraint', fontsize=12)
    ax1.axis('equal'); ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right')
    
    margin = 0.5
    ax1.set_xlim(path_points[:, 0].min() - margin, path_points[:, 0].max() + margin)
    ax1.set_ylim(path_points[:, 1].min() - margin, path_points[:, 1].max() + margin)
    
    # ---- 速度曲线 ----
    ax2 = plt.subplot2grid((2, 4), (0, 2))
    line_vel, = ax2.plot([], [], 'b-', lw=2, label='Velocity')
    ax2.axhline(v_max_final, color='r', ls='--', alpha=0.5, label=f'Vmax={v_max_final:.2f}')
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('S-curve Velocity')
    ax2.grid(True, alpha=0.3); ax2.legend(loc='upper right')
    ax2.set_xlim(0, total_time * 1.05)
    ax2.set_ylim(-0.1, v_max_final * 1.3)
    curr_vel_dot, = ax2.plot([], [], 'ro', ms=8)
    curr_vel_line = ax2.axvline(x=0, color='gray', alpha=0.5, ls='--')
    
    # ---- 角速度曲线 ----
    ax3 = plt.subplot2grid((2, 4), (1, 2))
    line_ang, = ax3.plot([], [], 'g-', lw=2, label='Angular Vel')
    ax3.axhline(0, color='k', alpha=0.3)
    ax3.axhline(omega_max, color='r', ls='--', alpha=0.5, label=f'ωmax={omega_max}')
    ax3.axhline(-omega_max, color='r', ls='--', alpha=0.5)
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel('Omega (rad/s)')
    ax3.set_title('Angular Velocity (Continuous)')
    ax3.grid(True, alpha=0.3); ax3.legend(loc='upper right')
    ax3.set_xlim(0, total_time * 1.05)
    max_omega = max(omega_max, np.max(np.abs(angular_vels)) * 1.2)
    ax3.set_ylim(-max_omega, max_omega)
    curr_ang_dot, = ax3.plot([], [], 'ro', ms=8)
    curr_ang_line = ax3.axvline(x=0, color='gray', alpha=0.5, ls='--')
    
    # ---- 曲率曲线 ----
    ax4 = plt.subplot2grid((2, 4), (0, 3))
    line_curv, = ax4.plot([], [], 'm-', lw=2, label='Curvature')
    ax4.axhline(0, color='k', alpha=0.3)
    ax4.set_xlabel('Time (s)'); ax4.set_ylabel('Curvature (1/m)')
    ax4.set_title('Path Curvature')
    ax4.grid(True, alpha=0.3); ax4.legend(loc='upper right')
    ax4.set_xlim(0, total_time * 1.05)
    max_kappa_plot = max(1.0, np.max(np.abs(curvatures_at_points)) * 1.2)
    ax4.set_ylim(-max_kappa_plot, max_kappa_plot)
    curr_curv_dot, = ax4.plot([], [], 'ro', ms=8)
    curr_curv_line = ax4.axvline(x=0, color='gray', alpha=0.5, ls='--')
    
    # ---- 角速度 vs 线速度 ----
    ax5 = plt.subplot2grid((2, 4), (1, 3))
    ax5.scatter(velocities, angular_vels, c=times, cmap='coolwarm', s=5, alpha=0.6)
    ax5.set_xlabel('Velocity (m/s)'); ax5.set_ylabel('Omega (rad/s)')
    ax5.set_title('Velocity vs Angular Vel')
    ax5.grid(True, alpha=0.3)
    plt.colorbar(ax5.collections[0], ax=ax5, label='Time (s)')
    
    # ---- 初始化 ----
    def init():
        trail_line.set_data([], [])
        vehicle.set_xy((0, 0))
        vehicle.set_angle(0)
        pos_scatter.set_offsets(np.empty((0, 2)))
        line_vel.set_data([], [])
        line_ang.set_data([], [])
        line_curv.set_data([], [])
        curr_vel_dot.set_data([], [])
        curr_ang_dot.set_data([], [])
        curr_curv_dot.set_data([], [])
        info_text.set_text('')
        return (trail_line, vehicle, pos_scatter, line_vel, line_ang, line_curv,
                curr_vel_dot, curr_ang_dot, curr_curv_dot, info_text)
    
    # ---- 更新 ----
    def update(frame):
        idx = min(frame, len(positions) - 1)
        
        # 已走轨迹
        trail_line.set_data(positions[:idx+1, 0], positions[:idx+1, 1])
        
        # AGV位置和姿态
        x, y = positions[idx]
        heading = headings[idx]
        vehicle.set_xy((x - vehicle_length/2, y - vehicle_width/2))
        vehicle.set_angle(np.degrees(heading))
        
        # 速度颜色散点
        pos_scatter.set_offsets(positions[:idx+1])
        pos_scatter.set_array(velocities[:idx+1])
        
        # 速度曲线
        t_data = times[:idx+1]
        line_vel.set_data(t_data, velocities[:idx+1])
        curr_vel_dot.set_data([times[idx]], [velocities[idx]])
        curr_vel_line.set_xdata([times[idx]])
        
        # 角速度曲线
        line_ang.set_data(t_data, angular_vels[:idx+1])
        curr_ang_dot.set_data([times[idx]], [angular_vels[idx]])
        curr_ang_line.set_xdata([times[idx]])
        
        # 曲率曲线
        line_curv.set_data(t_data, curvatures_at_points[:idx+1])
        curr_curv_dot.set_data([times[idx]], [curvatures_at_points[idx]])
        curr_curv_line.set_xdata([times[idx]])
        
        # 信息
        info_text.set_text(
            f'Time: {times[idx]:.2f}s\n'
            f'Pos: ({x:.2f}, {y:.2f})m\n'
            f'Velocity: {velocities[idx]:.2f} m/s\n'
            f'Omega: {angular_vels[idx]:.2f} rad/s\n'
            f'Curvature: {curvatures_at_points[idx]:.2f} 1/m\n'
            f'Progress: {displacements[idx]/total_length*100:.1f}%'
        )
        
        return (trail_line, vehicle, pos_scatter, line_vel, line_ang, line_curv,
                curr_vel_dot, curr_ang_dot, curr_curv_dot, info_text)
    
    # ---- 运行动画 ----
    step = max(1, len(positions) // 300)
    frames = range(0, len(positions), step)
    
    ani = animation.FuncAnimation(
        fig, update, frames=frames,
        init_func=init, blit=True, interval=30, repeat=True
    )
    
    plt.tight_layout()
    plt.show()
    
    return ani

if __name__ == "__main__":
    plt.rcParams['font.sans-serif'] = ['WenQuanYi Zen Hei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    ani = main()