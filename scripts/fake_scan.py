#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, math, random, time
from sensor_msgs.msg import LaserScan

# 3 m x 3 m 로컬 공간 (로봇은 정중앙: (0,0), 방 형태)
HALF = 1.5
X_MIN, X_MAX = -HALF, HALF
Y_MIN, Y_MAX = -HALF, HALF
RANGE_MAX = 3.2  # 살짝 여유

def clamp(v, lo, hi): return lo if v<lo else hi if v>hi else v

def ray_hit_hline(yline, ang):
    s, c = math.sin(ang), math.cos(ang)
    if abs(s) < 1e-6: return None
    t = yline / s
    if t <= 0: return None
    x = c * t
    return t if (X_MIN <= x <= X_MAX) else None

def ray_hit_vline(xline, ang):
    s, c = math.sin(ang), math.cos(ang)
    if abs(c) < 1e-6: return None
    t = xline / c
    if t <= 0: return None
    y = s * t
    return t if (Y_MIN <= y <= Y_MAX) else None

def ray_hit_circle(cx, cy, r, ang):
    s, c = math.sin(ang), math.cos(ang)
    b = -2.0 * (c*cx + s*cy)
    cc = cx*cx + cy*cy - r*r
    disc = b*b - 4.0*cc
    if disc < 0: return None
    sd = math.sqrt(disc)
    t1 = (-b - sd) / 2.0
    t2 = (-b + sd) / 2.0
    cand = [t for t in (t1, t2) if t>0]
    return min(cand) if cand else None

class Walker(object):
    """사람 보행 모델: 연속 속도·가끔 정지·방향 소폭 변동·벽 반사"""
    def __init__(self, x, y, vx, vy, r=0.25):
        self.x, self.y = x, y
        self.vx, self.vy = vx, vy
        self.r = r
        self.t_stop = 0.0

    def step(self, dt):
        if self.t_stop > 0.0:
            self.t_stop -= dt
            self.vx *= 0.98; self.vy *= 0.98
        else:
            if random.random() < 0.01:               # 가끔 멈춤
                self.t_stop = random.uniform(0.2, 0.6)
            v = math.hypot(self.vx, self.vy)         # 서서히 가감속
            v += random.uniform(-0.05, 0.05)
            v = clamp(v, 0.3, 1.2)
            ang = math.atan2(self.vy, self.vx) + random.uniform(-0.07, 0.07)
            self.vx, self.vy = v*math.cos(ang), v*math.sin(ang)

        self.x += self.vx * dt
        self.y += self.vy * dt

        # 벽 반사
        if self.x < X_MIN + self.r: self.x = X_MIN + self.r; self.vx = abs(self.vx)
        if self.x > X_MAX - self.r: self.x = X_MAX - self.r; self.vx = -abs(self.vx)
        if self.y < Y_MIN + self.r: self.y = Y_MIN + self.r; self.vy = abs(self.vy)
        if self.y > Y_MAX - self.r: self.y = Y_MAX - self.r; self.vy = -abs(self.vy)

def main():
    rospy.init_node('fake_scan')
    pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
    rate_hz = 20.0
    rate = rospy.Rate(rate_hz)

    # FOV = 180° (−90° ~ +90°)
    angle_min = math.radians(-90.0)
    angle_max = math.radians( 90.0)
    N = 540                                # 0.333° 분해능
    dth = (angle_max - angle_min)/(N-1)
    rmin, rmax = 0.05, RANGE_MAX

    # 보행자 3명 (초기 위치/속도 서로 다르게)
    walkers = [
        Walker(-0.8,  0.0,  0.8,  0.1),
        Walker( 0.9, -0.6, -0.6,  0.7),
        Walker( 0.5,  0.8,  0.4, -0.9),
    ]

    last = time.time()
    while not rospy.is_shutdown():
        now = time.time()
        dt = max(1.0/rate_hz, now - last); last = now

        for w in walkers: w.step(dt)

        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'           # 로봇 프레임 고정
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = dth
        msg.time_increment = 0.0
        msg.scan_time = 1.0/rate_hz
        msg.range_min = rmin
        msg.range_max = rmax

        ranges = [rmax]*N
        for i in range(N):
            ang = angle_min + i*dth
            best = rmax
            # 방 네 벽
            for yline in (Y_MIN, Y_MAX):
                t = ray_hit_hline(yline, ang);  best = min(best, t) if t is not None else best
            for xline in (X_MIN, X_MAX):
                t = ray_hit_vline(xline, ang);  best = min(best, t) if t is not None else best
            # 보행자
            for w in walkers:
                t = ray_hit_circle(w.x, w.y, w.r, ang)
                if t is not None: best = min(best, t)

            # 매우 약한 노이즈 → 스캔이 '뭉쳐' 보이도록
            ranges[i] = clamp(best + random.uniform(-0.003, 0.003), rmin, rmax)

        msg.ranges = ranges
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
