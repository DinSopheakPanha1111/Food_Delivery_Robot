#!/usr/bin/env python3
"""
dwa_sim.py — draggable obstacle
Click and drag the red obstacle anywhere on the map.
Robot reacts in real time.
"""

import math
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ─────────────────────────────────────────────
#  Parameters
# ─────────────────────────────────────────────
MAX_LINEAR_VEL    = 0.10
MIN_LINEAR_VEL    = 0.05
MAX_ANGULAR_VEL   = 0.50
MAX_LINEAR_ACCEL  = 1.0
MAX_ANGULAR_ACCEL = 1.0
ROBOT_RADIUS      = 0.22
ROBOT_SIZE        = 0.22
DT                = 0.10
WINDOW_TIME       = 3.0    # ← plan farther ahead
CONTROL_DT        = 0.05
VEL_RESOLUTION    = 0.02
ANG_RESOLUTION    = 0.05
GOAL_COST_FACTOR      = 1.0
OBSTACLE_COST_FACTOR  = 8.0   # ← stronger obstacle repulsion
VELOCITY_COST_FACTOR  = 0.1
XY_GOAL_TOL = 0.35
LOOKAHEAD   = 3.0
OBS_RADIUS  = 0.80            # ← inflated safety bubble
CLEARANCE_THRESH = 3.0        # ← start dodging at 3m

# ─────────────────────────────────────────────
#  Kinematics
# ─────────────────────────────────────────────
def predict_state(state, control, dt):
    x,y,theta,v,w = state
    theta += control[1]*dt
    x     += control[0]*math.cos(theta)*dt
    y     += control[0]*math.sin(theta)*dt
    return [x,y,theta,control[0],control[1]]

def dynamic_window(state, dt):
    cv,cw = state[3],state[4]
    li=MAX_LINEAR_ACCEL*dt; ai=MAX_ANGULAR_ACCEL*dt
    return [max(cv-li,MIN_LINEAR_VEL), min(cv+li,MAX_LINEAR_VEL),
            max(cw-ai,-MAX_ANGULAR_VEL), min(cw+ai,MAX_ANGULAR_VEL)]

def calculate_trajectory(state, control, dt, window_time):
    traj=[state[:]]; s=state[:]; t=0.0
    while t<=window_time:
        s=predict_state(s,control,dt); traj.append(s[:]); t+=dt
    return traj

# ─────────────────────────────────────────────
#  Cost functions
# ─────────────────────────────────────────────
def goal_cost(traj, goal):
    ex,ey,eth=traj[-1][0],traj[-1][1],traj[-1][2]; gx,gy=goal
    a2g=math.atan2(gy-ey,gx-ex)
    return GOAL_COST_FACTOR*abs(math.atan2(math.sin(a2g-eth),math.cos(a2g-eth)))

def obstacle_cost(traj, obstacles):
    min_sep=float("inf")
    for i in range(len(traj)):
        for (ox,oy,orad) in obstacles:
            dx=traj[i][0]-ox; dy=traj[i][1]-oy
            sep=math.sqrt(dx*dx+dy*dy)-orad-ROBOT_RADIUS
            if sep<=0: return 1e6
            if sep<min_sep: min_sep=sep
    if min_sep==float("inf"): return 0.0
    return OBSTACLE_COST_FACTOR/min_sep

def velocity_cost(traj, obstacles):
    min_sep=float("inf")
    for s in traj:
        for (ox,oy,orad) in obstacles:
            sep=math.sqrt((s[0]-ox)**2+(s[1]-oy)**2)-orad-ROBOT_RADIUS
            if sep<min_sep: min_sep=sep
    if min_sep>CLEARANCE_THRESH:
        return VELOCITY_COST_FACTOR*(MAX_LINEAR_VEL-traj[-1][3])
    return 0.0

def rollout_trajectories(state, goal, obstacles):
    v_lo,v_hi,w_lo,w_hi=dynamic_window(state,DT)
    min_cost=float("inf"); best_traj=None; best_ctrl=(MIN_LINEAR_VEL,0.0)
    v=v_lo
    while v<=v_hi+1e-6:
        w=w_lo
        while w<=w_hi+1e-6:
            traj=calculate_trajectory(state,(v,w),DT,WINDOW_TIME)
            cost=(goal_cost(traj,goal)
                 +obstacle_cost(traj,obstacles)
                 +velocity_cost(traj,obstacles))
            if cost<min_cost: min_cost=cost; best_traj=traj; best_ctrl=(v,w)
            w+=ANG_RESOLUTION
        v+=VEL_RESOLUTION
    if best_traj is None:
        best_ctrl=(MIN_LINEAR_VEL,0.0)
        best_traj=calculate_trajectory(state,best_ctrl,DT,WINDOW_TIME)
    return best_traj,best_ctrl

# ─────────────────────────────────────────────
#  Path
# ─────────────────────────────────────────────
WAYPOINTS=[(1.0,1.0),(3.0,4.0),(6.0,2.5),(8.0,6.0),(5.5,8.5),(9.0,10.5),(10.5,11.0)]
GOAL=WAYPOINTS[-1]

def make_path(waypoints,n=80):
    path=[]
    for i in range(len(waypoints)-1):
        x0,y0=waypoints[i]; x1,y1=waypoints[i+1]
        yaw=math.atan2(y1-y0,x1-x0)
        for s in range(n):
            t=s/n; path.append((x0+t*(x1-x0),y0+t*(y1-y0),yaw))
    yaw_l=math.atan2(waypoints[-1][1]-waypoints[-2][1],waypoints[-1][0]-waypoints[-2][0])
    path.append((*waypoints[-1],yaw_l))
    return path

GLOBAL_PATH=make_path(WAYPOINTS)

def get_lookahead(rx,ry):
    best_d,best_i=float("inf"),0
    for i,(px,py,_) in enumerate(GLOBAL_PATH):
        dd=math.sqrt((rx-px)**2+(ry-py)**2)
        if dd<best_d: best_d,best_i=dd,i
    for i in range(best_i,len(GLOBAL_PATH)):
        px,py,_=GLOBAL_PATH[i]
        if math.sqrt((rx-px)**2+(ry-py)**2)>=LOOKAHEAD: return px,py
    return GLOBAL_PATH[-1][0],GLOBAL_PATH[-1][1]

# ─────────────────────────────────────────────
#  Draggable obstacle state
# ─────────────────────────────────────────────
OBS_IDX=int(len(GLOBAL_PATH)*0.35)
obs_pos=[GLOBAL_PATH[OBS_IDX][0], GLOBAL_PATH[OBS_IDX][1]]
dragging=False

# ─────────────────────────────────────────────
#  Robot state
# ─────────────────────────────────────────────
robot=[WAYPOINTS[0][0],WAYPOINTS[0][1],
       math.atan2(WAYPOINTS[1][1]-WAYPOINTS[0][1],
                  WAYPOINTS[1][0]-WAYPOINTS[0][0]),0.0,0.0]
reached=False; trail_x=[robot[0]]; trail_y=[robot[1]]

# ─────────────────────────────────────────────
#  Plot
# ─────────────────────────────────────────────
fig,ax=plt.subplots(figsize=(9,9))
ax.set_xlim(-0.5,12); ax.set_ylim(-0.5,12); ax.set_aspect("equal")
ax.set_facecolor("#1a1a2e"); fig.patch.set_facecolor("#1a1a2e")
ax.tick_params(colors="#aaaaaa")
for sp in ax.spines.values(): sp.set_edgecolor("#444444")
ax.set_title("DWA — drag the red obstacle to test avoidance",color="white",fontsize=12)
ax.set_xlabel("x [m]",color="#aaaaaa"); ax.set_ylabel("y [m]",color="#aaaaaa")
ax.grid(True,color="#2a2a4a",linewidth=0.5,zorder=0)

ax.plot([p[0] for p in GLOBAL_PATH],[p[1] for p in GLOBAL_PATH],
        "-",color="#3498db",lw=2.5,alpha=0.8,zorder=2,label="global path")
for wx,wy in WAYPOINTS[1:-1]: ax.plot(wx,wy,"D",color="#3498db",markersize=7,zorder=3)
ax.text(WAYPOINTS[0][0]+.1,WAYPOINTS[0][1]-.4,"START",color="#2ecc71",fontsize=9,fontweight="bold",zorder=5)
ax.plot(*GOAL,"*",color="#f1c40f",markersize=22,zorder=6,label="goal")
ax.add_patch(plt.Circle(GOAL,XY_GOAL_TOL,color="#f1c40f",alpha=0.2,zorder=2))

obs_patch=plt.Circle(tuple(obs_pos),OBS_RADIUS,color="#e74c3c",alpha=0.92,zorder=5,label="obstacle (drag me)")
ax.add_patch(obs_patch)
obs_lbl=ax.text(obs_pos[0],obs_pos[1]+OBS_RADIUS+0.25,"⬆ drag me",color="white",fontsize=9,ha="center",zorder=6,fontweight="bold")

trail_line,=ax.plot([],[],"-",color="#2ecc71",lw=2.,alpha=.9,zorder=4,label="robot trail")
traj_line, =ax.plot([],[],"-",color="#f39c12",lw=1.5,alpha=.8,zorder=4,label="best trajectory")
robot_poly =plt.Polygon([(0,0)]*4,closed=True,lw=2.,edgecolor="#00d4ff",facecolor="#003355",zorder=7,label="robot")
ax.add_patch(robot_poly)
arrow_obj=ax.annotate("",xy=(1,0),xytext=(0,0),arrowprops=dict(arrowstyle="-|>",color="#00d4ff",lw=2.5),zorder=8)

status_txt=ax.text(.02,.97,"",transform=ax.transAxes,color="white",fontsize=9,va="top",fontfamily="monospace")
mode_txt  =ax.text(.02,.90,"",transform=ax.transAxes,fontsize=11,va="top",fontweight="bold",fontfamily="monospace")
hint_txt  =ax.text(.98,.97,"",transform=ax.transAxes,color="#aaaaaa",fontsize=8,va="top",ha="right",fontfamily="monospace")
ax.legend(loc="lower right",facecolor="#222244",labelcolor="white",fontsize=8)

# ─────────────────────────────────────────────
#  Mouse drag handlers
# ─────────────────────────────────────────────
def on_press(event):
    global dragging
    if event.inaxes!=ax: return
    ox,oy=obs_pos
    dist=math.sqrt((event.xdata-ox)**2+(event.ydata-oy)**2)
    if dist<=OBS_RADIUS*1.5: dragging=True

def on_motion(event):
    if not dragging: return
    if event.inaxes!=ax: return
    obs_pos[0]=event.xdata; obs_pos[1]=event.ydata
    obs_patch.set_center(tuple(obs_pos))
    obs_lbl.set_position((obs_pos[0], obs_pos[1]+OBS_RADIUS+0.25))

def on_release(event):
    global dragging
    dragging=False

fig.canvas.mpl_connect("button_press_event",   on_press)
fig.canvas.mpl_connect("motion_notify_event",  on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)

# ─────────────────────────────────────────────
#  Robot corners
# ─────────────────────────────────────────────
def rcorners(x,y,yaw):
    s=ROBOT_SIZE; c,si=math.cos(yaw),math.sin(yaw)
    return [(c*cx-si*cy+x,si*cx+c*cy+y) for cx,cy in [(-s,-s),(s,-s),(s,s),(-s,s)]]

# ─────────────────────────────────────────────
#  Animation
# ─────────────────────────────────────────────
def update(_):
    global robot,reached,trail_x,trail_y

    obstacle=[(obs_pos[0],obs_pos[1],OBS_RADIUS)]

    if reached:
        status_txt.set_text("✓ Goal reached!  Drag obstacle onto path to retest.")
        mode_txt.set_text("[ DONE ]"); mode_txt.set_color("#2ecc71")
        return (robot_poly,trail_line,traj_line,arrow_obj,status_txt,mode_txt,obs_patch,obs_lbl)

    rx,ry=robot[0],robot[1]; gx,gy=GOAL
    dg  =math.sqrt((rx-gx)**2+(ry-gy)**2)
    dobs=math.sqrt((rx-obs_pos[0])**2+(ry-obs_pos[1])**2)-OBS_RADIUS

    if dg<XY_GOAL_TOL:
        reached=True; traj_xs=traj_ys=[]; mode,col="DONE","#2ecc71"
    else:
        lx,ly=get_lookahead(rx,ry)
        best_traj,best_ctrl=rollout_trajectories(robot,(lx,ly),obstacle)
        robot=predict_state(robot,best_ctrl,CONTROL_DT)
        trail_x.append(robot[0]); trail_y.append(robot[1])
        traj_xs=[s[0] for s in best_traj]
        traj_ys=[s[1] for s in best_traj]

        if dobs<0.5:   mode,col="AVOIDING","#e74c3c"
        elif dobs<1.2: mode,col="NEAR OBS","#f39c12"
        else:           mode,col="FOLLOW","#2ecc71"

    robot_poly.set_xy(rcorners(robot[0],robot[1],robot[2]))
    tx=robot[0]+.6*math.cos(robot[2]); ty=robot[1]+.6*math.sin(robot[2])
    arrow_obj.xytext=(robot[0],robot[1]); arrow_obj.xy=(tx,ty)
    trail_line.set_data(trail_x,trail_y)
    traj_line.set_data(traj_xs,traj_ys)

    hint_txt.set_text("drag red circle\nto move obstacle")
    status_txt.set_text(
        f"pos({robot[0]:5.2f},{robot[1]:5.2f})  yaw {math.degrees(robot[2])%360:.1f}°\n"
        f"v={robot[3]:.3f}m/s  w={robot[4]:.3f}rad/s  "
        f"dist_goal={dg:.2f}  clearance={dobs:.2f}m")
    mode_txt.set_text(f"[ {mode} ]"); mode_txt.set_color(col)

    return (robot_poly,trail_line,traj_line,arrow_obj,
            status_txt,mode_txt,obs_patch,obs_lbl,hint_txt)


anim=FuncAnimation(fig,update,interval=50,blit=False,cache_frame_data=False)
plt.tight_layout()
plt.show()