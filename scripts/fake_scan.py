#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, math, random, time
from sensor_msgs.msg import LaserScan

# 3 m x 3 m 로컬 공간
HALF = 1.5
X_MIN, X_MAX = -HALF, HALF
Y_MIN, Y_MAX = -HALF, HALF
RANGE_MAX = 3.2

def clamp(v, lo, hi): return lo if v<lo else hi if v>hi else v

# --- (Ray-casting 함수들은 기존과 동일) ---
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

# --- [핵심 수정] 테스트 시나리오를 위한 Walker ---
class TestWalker(object):
    """
    (0,0) 함정 및 측면 이동 테스트를 위한 상태 기반 Walker
    시나리오: 
    1. STATIC: 5초간 (0.8, -0.8)에서 정지 (함정 유발)
    2. FORWARD: 2초간 전진 (vr != 0)
    3. STATIC: 5초간 (0.8, 0.8)에서 정지 (함정 유발)
    4. CROSSING: 2초간 측면 이동 (vr == 0)
    """
    STATE_STATIC_1 = 0
    STATE_FORWARD  = 1
    STATE_STATIC_2 = 2
    STATE_CROSSING = 3
    
    def __init__(self, r=0.25):
        self.r = r
        self.state = self.STATE_STATIC_1
        self.timer = 5.0 # 5초간 정지
        self.x, self.y = 0.8, -0.8
        self.vx, self.vy = 0.0, 0.0
        rospy.loginfo("TestWalker: 5초간 [정지] (함정 유발)")

    def step(self, dt):
        self.timer -= dt

        if self.state == self.STATE_STATIC_1:
            if self.timer <= 0:
                self.state = self.STATE_FORWARD
                self.timer = 2.0 # 2초간 전진
                self.vx, self.vy = 0.0, 0.8 # 전진 (y방향 +)
                rospy.loginfo("TestWalker: 2초간 [전진] (range_rate != 0)")
        
        elif self.state == self.STATE_FORWARD:
            if self.timer <= 0:
                self.state = self.STATE_STATIC_2
                self.timer = 5.0 # 5초간 정지
                self.x, self.y = 0.8, 0.8 # 새 위치에서 정지
                self.vx, self.vy = 0.0, 0.0
                rospy.loginfo("TestWalker: 5초간 [정지] (함정 유발)")

        elif self.state == self.STATE_STATIC_2:
            if self.timer <= 0:
                self.state = self.STATE_CROSSING
                self.timer = 2.0 # 2초간 측면 이동
                self.vx, self.vy = -0.8, 0.0 # 측면 이동 (x방향 -)
                rospy.loginfo("TestWalker: 2초간 [측면 이동] (range_rate == 0)")
        
        elif self.state == self.STATE_CROSSING:
            if self.timer <= 0:
                self.state = self.STATE_STATIC_1
                self.timer = 5.0 # 5초간 정지
                self.x, self.y = -0.8, 0.8 # 새 위치에서 정지
                self.vx, self.vy = 0.0, 0.0
                rospy.loginfo("TestWalker: 5초간 [정지] (함정 유발)")

        # 위치 업데이트
        self.x += self.vx * dt
        self.y += self.vy * dt
        # 벽 반사는 단순화를 위해 제거 (시나리오가 꼬이지 않도록)


def main():
    rospy.init_node('fake_scan')
    pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
    rate_hz = 20.0
    rate = rospy.Rate(rate_hz)

    # FOV = 180° (−90° ~ +90°)
    angle_min = math.radians(-90.0)
    angle_max = math.radians( 90.0)
    N = 540
    dth = (angle_max - angle_min)/(N-1)
    rmin, rmax = 0.05, RANGE_MAX

    # [수정] 1개의 테스트용 Walker만 사용
    walker = TestWalker()

    last = time.time()
    while not rospy.is_shutdown():
        now = time.time()
        dt = max(1.0/rate_hz, now - last); last = now

        walker.step(dt) # Walker 상태 업데이트

        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
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
            
            # [수정] 1개의 Walker만 계산
            t = ray_hit_circle(walker.x, walker.y, walker.r, ang)
            if t is not None: best = min(best, t)

            ranges[i] = clamp(best + random.uniform(-0.003, 0.003), rmin, rmax)

        msg.ranges = ranges
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()