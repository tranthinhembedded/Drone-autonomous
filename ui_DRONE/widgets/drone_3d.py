import time
from math import sin, cos, radians
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtCore import QTimer
from OpenGL.GL import *
from OpenGL.GLU import *

class Drone3D(QOpenGLWidget):
    def __init__(self):
        super().__init__()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.rotor_angle = 0.0
        self.motor_speeds = [1000, 1000, 1000, 1000]

        # Camera
        self.cam_dist = 7.0
        self.cam_elev = radians(35)
        self.cam_azim = radians(-45)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_animation)
        self.timer.start(16)

    def update_animation(self):
        # Quay cánh nhanh chậm tùy ga
        avg = sum(self.motor_speeds) / 4
        spin = (avg - 1000) / 20 + 10
        self.rotor_angle -= spin
        if self.rotor_angle < 0: self.rotor_angle += 360
        self.update()

    def set_angles(self, r, p, y):
        self.roll = r; self.pitch = p; self.yaw = y
        self.update()

    def set_motor_speeds(self, speeds):
        if len(speeds) == 4:
            self.motor_speeds = speeds
            self.update()

    def initializeGL(self):
        glClearColor(0.05, 0.06, 0.09, 1.0)
        glEnable(GL_DEPTH_TEST); glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_LINE_SMOOTH)

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        gluPerspective(50, w / (h if h > 0 else 1), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); glLoadIdentity()

        dx = self.cam_dist * cos(self.cam_elev) * cos(self.cam_azim)
        dy = self.cam_dist * sin(self.cam_elev)
        dz = self.cam_dist * cos(self.cam_elev) * sin(self.cam_azim)
        gluLookAt(dx, dy, dz, 0, 0, 0, 0, 1, 0)

        self.draw_neon_grid()

        # XOAY DRONE
        glPushMatrix()
        # Yaw (Quanh Y)
        glRotatef(self.yaw, 0, 1, 0)
        # Pitch (Quanh X - Đỏ)
        glRotatef(self.pitch, 1, 0, 0)
        # Roll (Quanh Z - Xanh Dương)
        glRotatef(self.roll, 0, 0, 1)

        self.draw_drone_quad_x()
        glPopMatrix()

    def draw_neon_grid(self):
        glLineWidth(1); glBegin(GL_LINES); glColor4f(0.2, 0.95, 0.85, 0.15)
        for i in range(-10, 11):
            glVertex3f(i, -1, -10); glVertex3f(i, -1, 10)
            glVertex3f(-10, -1, i); glVertex3f(10, -1, i)
        glEnd()

    def draw_drone_quad_x(self):
        # 1. Vẽ Trục tọa độ
        self.draw_axis_guides()

        # 2. Vẽ Thân Drone
        glPushMatrix()
        glScalef(0.6, 0.1, 0.6)
        glColor4f(0.15, 0.16, 0.20, 0.9); self.draw_box_solid()
        glLineWidth(1.5); glColor3f(0.0, 0.8, 1.0); self.draw_box_wire()
        glPopMatrix()

        # 3. Vẽ Mũi tên hướng (Trục Z - Xanh Dương là đầu)
        glLineWidth(4); glBegin(GL_LINES)
        glColor3f(0.0, 0.5, 1.0) 
        glVertex3f(0, 0, 0); glVertex3f(0, 0, 2.5) 
        glEnd()
        # Đầu mũi tên
        glPushMatrix(); glTranslatef(0, 0, 2.5)
        glColor3f(0.0, 0.8, 1.0); gluSphere(gluNewQuadric(), 0.1, 8, 8)
        glPopMatrix()

        # --- 4. CẤU HÌNH VỊ TRÍ MOTOR (FIX CỨNG) ---
        # Quy ước: X+ là Phải (Right), Z+ là Trước (Front)
        # M1: Front Right (Trước Phải)
        # M2: Back Right  (Sau Phải)
        # M3: Back Left   (Sau Trái)
        # M4: Front Left  (Trước Trái)
        
        motor_layout = [
            # x,    z,    is_front, spin, idx
            ( -1.2,  1.2,  True,    -1,    0), # M1: Front Right (CW)
            ( -1.2, -1.2,  False,    1,    1), # M2: Back Right  (CCW)
            (1.2, -1.2,  False,   -1,    2), # M3: Back Left   (CW)
            (1.2,  1.2,  True,     1,    3)  # M4: Front Left  (CCW)
        ]

        for x, z, is_front, spin, m_idx in motor_layout:
            glPushMatrix()
            
            # A. Vẽ cánh tay nối từ tâm ra motor
            glLineWidth(6)
            glBegin(GL_LINES)
            if is_front: glColor3f(1.0, 0.5, 0.0) # Cam (Tay trước)
            else:        glColor3f(0.3, 0.3, 0.3) # Xám (Tay sau)
            glVertex3f(0, 0, 0); glVertex3f(x, 0, z)
            glEnd()

            # B. Di chuyển đến vị trí đầu cánh tay
            glTranslatef(x, 0.05, z)
            
            # C. Vẽ Motor & Cánh quạt
            self.draw_motor_and_prop(is_front, spin)
            
            # D. Vẽ Cột lực (Force Bar) dựa trên tốc độ thực tế
            current_speed = self.motor_speeds[m_idx]
            self.draw_force_bar(current_speed)
            
            glPopMatrix()

    def draw_force_bar(self, speed):
        ratio = max(0.0, (speed - 1000)/1000.0)
        if ratio < 0.05: return
        h = ratio * 3.0
        r, g = min(1.0, ratio*2), min(1.0, (1.0-ratio)*2)
        
        q = gluNewQuadric()
        glPushMatrix(); glTranslatef(0, 0.2, 0); glRotatef(-90, 1, 0, 0)
        glColor4f(r, g, 0, 0.6); gluCylinder(q, 0.12, 0.05, h, 12, 1)
        glTranslatef(0,0,h); gluDisk(q,0,0.05,12,1)
        glPopMatrix(); gluDeleteQuadric(q)

    def draw_axis_guides(self):
        glLineWidth(2.5)
        # X (RED) = PITCH
        glBegin(GL_LINES); glColor3f(1, 0.2, 0.2); glVertex3f(-4,0,0); glVertex3f(4,0,0); glEnd()
        glPushMatrix(); glTranslatef(4.2,0,0); glScalef(0.3,0.3,0.3); self.draw_char_P((1,0.2,0.2)); glPopMatrix()

        # Z (BLUE) = ROLL
        glBegin(GL_LINES); glColor3f(0.2, 0.5, 1); glVertex3f(0,0,-4); glVertex3f(0,0,4); glEnd()
        glPushMatrix(); glTranslatef(0,0,4.2); glRotatef(90,0,1,0); glScalef(0.3,0.3,0.3); self.draw_char_R((0.2,0.5,1)); glPopMatrix()

        # Y (GREEN) = YAW
        glBegin(GL_LINES); glColor3f(0.2, 1, 0.2); glVertex3f(0,-2,0); glVertex3f(0,2,0); glEnd()
        glPushMatrix(); glTranslatef(0,2.3,0); glScalef(0.3,0.3,0.3); self.draw_char_Y((0.2,1,0.2)); glPopMatrix()

    # Chữ cái
    def draw_char_R(self, c):
        glColor3f(*c); glLineWidth(2); glBegin(GL_LINE_STRIP)
        glVertex3f(0,0,0); glVertex3f(0,1,0); glVertex3f(0.8,1,0); glVertex3f(0.8,0.5,0); glVertex3f(0,0.5,0); glEnd()
        glBegin(GL_LINES); glVertex3f(0.2,0.5,0); glVertex3f(0.8,0,0); glEnd()
    def draw_char_P(self, c):
        glColor3f(*c); glLineWidth(2); glBegin(GL_LINE_STRIP)
        glVertex3f(0,0,0); glVertex3f(0,1,0); glVertex3f(0.8,1,0); glVertex3f(0.8,0.5,0); glVertex3f(0,0.5,0); glEnd()
    def draw_char_Y(self, c):
        glColor3f(*c); glLineWidth(2); glBegin(GL_LINE_STRIP)
        glVertex3f(0,1,0); glVertex3f(0.5,0.5,0); glVertex3f(0.5,0,0); glVertex3f(0.5,0.5,0); glVertex3f(1,1,0); glEnd()

    def draw_motor_and_prop(self, is_f, spin):
        q = gluNewQuadric()
        glColor3f(0.3, 0.3, 0.35)
        glPushMatrix(); glRotatef(90, 1, 0, 0); gluCylinder(q, 0.2, 0.2, 0.15, 16, 1); glPopMatrix()
        glPushMatrix(); glTranslatef(0, 0.15, 0); glRotatef(self.rotor_angle*spin, 0, 1, 0)
        if is_f: glColor4f(1.0, 0.5, 0.0, 0.5)
        else:    glColor4f(0.0, 0.9, 1.0, 0.5)
        glPushMatrix(); glRotatef(90, 1, 0, 0); gluDisk(q, 0, 1.0, 20, 1); glPopMatrix()
        glPopMatrix(); gluDeleteQuadric(q)

    def draw_box_solid(self):
        glBegin(GL_QUADS)
        glVertex3f(-0.5,0.5,0.5); glVertex3f(0.5,0.5,0.5); glVertex3f(0.5,0.5,-0.5); glVertex3f(-0.5,0.5,-0.5)
        glVertex3f(-0.5,-0.5,0.5); glVertex3f(-0.5,-0.5,-0.5); glVertex3f(0.5,-0.5,-0.5); glVertex3f(0.5,-0.5,0.5)
        glVertex3f(0.5,-0.5,0.5); glVertex3f(0.5,-0.5,-0.5); glVertex3f(0.5,0.5,-0.5); glVertex3f(0.5,0.5,0.5)
        glVertex3f(-0.5,-0.5,0.5); glVertex3f(-0.5,0.5,0.5); glVertex3f(-0.5,0.5,-0.5); glVertex3f(-0.5,-0.5,-0.5)
        glEnd()
    def draw_box_wire(self):
        glBegin(GL_LINE_LOOP); glVertex3f(-0.5,0.5,0.5); glVertex3f(0.5,0.5,0.5); glVertex3f(0.5,0.5,-0.5); glVertex3f(-0.5,0.5,-0.5); glEnd()
        glBegin(GL_LINE_LOOP); glVertex3f(-0.5,-0.5,0.5); glVertex3f(0.5,-0.5,0.5); glVertex3f(0.5,-0.5,-0.5); glVertex3f(-0.5,-0.5,-0.5); glEnd()
        glBegin(GL_LINES); glVertex3f(-0.5,0.5,0.5); glVertex3f(-0.5,-0.5,0.5); glVertex3f(0.5,0.5,0.5); glVertex3f(0.5,-0.5,0.5); glVertex3f(0.5,0.5,-0.5); glVertex3f(0.5,-0.5,-0.5); glVertex3f(-0.5,0.5,-0.5); glVertex3f(-0.5,-0.5,-0.5); glEnd()