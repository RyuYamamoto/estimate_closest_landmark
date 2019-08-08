# -*- coding: utf-8 -*-
import almath
import yaml
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

class Visualize:
    def __init__(self):
        self.r = 0.2
        self.robot_pos_x_list = []
        self.robot_pos_y_list = []
        self.way_point_x = []
        self.way_point_y = []

        self.fig = plt.figure()
        self.ims = []
        self.ax = plt.axes()

    def config_screen(self):
        self.ax.cla()
        self.ax.axis('equal')
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(-6, 6)
        self.ax.set_xlabel("X [m]", fontsize=20)
        self.ax.set_ylabel("Y [m]", fontsize=20)

    def move_robot(self, pos):
        x, y, theta = pos

        # 座標系を描画
        self.draw_coordinate(pos)

        xn = x + self.r * math.cos(theta)
        yn = y + self.r * math.sin(theta)
        self.ax.plot([x, xn], [y, yn], color="black")
        c = patches.Circle(xy=(x, y), radius=self.r, fill=False, color="black")
        self.ax.add_patch(c)

    def draw_circle(self,x,y):
        c = patches.Circle(xy=(x, y), radius=0.5, fill=False, color="red")
        self.ax.add_patch(c)

    def draw_coordinate(self, pose):
        x, y, theta = pose
        ux = 0.3 * math.cos(theta)
        vx = 0.3 * math.sin(theta)
        self.ax.quiver(x, y, ux, vx, angles='xy',
                       scale_units='xy', alpha=0.3, width=0.003, scale=1)
        self.ax.annotate("x", xy=(x+ux, y+vx))

        uy = - 0.3 * math.sin(theta)
        vy = 0.3 * math.cos(theta)
        self.ax.quiver(x, y, uy, vy, angles='xy',
                       scale_units='xy', alpha=0.3, width=0.003, scale=1)
        self.ax.annotate("y", xy=(x + uy, y + vy))

    def draw_landmark(self, landmark_list):
        for id in landmark_list:
            x, y, _, _, _, yaw = landmark_list[id]
            pose = [x,y,yaw]
            self.draw_coordinate(pose)

class EstimateLandmark:
    def __init__(self):
        pass

    # ロボットの現在位置から一番近いランドマークを取得する
    # TODO まずは2次元平面で考える
    # 1. 現在位置から動くことなくかつ任意の範囲に収まる場所にランドマークがあるかを推定
    # 2. 1で存在する場合はその中から一番距離が近いランドマークを計算し取得
    #    存在しなかった場合は全ランドマークに対して距離を計算し一番距離が近いランドマークを返す
    #    (※)その中でも動く必要がある場合はなるべく移動量の少なくて済むランドマークを推定したい
    # TODO
    # ロボット、ランドマークそれぞれのベクトル向きを計算し、それらのなす角度の誤差を計算し一番誤差が少ないものを選択する？
    def get_closest_landmark_non_move(self, robot_pose, landmark):
        side_range = [-math.radians(70), math.radians(70)]
        min_distance = 100
        min_landmark_id = -1
        for id in landmark:
            # calculate angle robot to landmark
            # マップ原点座標から見たランドマークの位置・姿勢
            map2landmark = almath.Position6D(landmark[id])
            t_map2landmark = almath.transformFromPosition6D(map2landmark)
            # map原点座標から見たロボットの位置・姿勢
            map2robot = almath.position6DFromPose2D(robot_pose)
            t_map2robot = almath.transformFromPosition6D(map2robot)
            # ロボットから見たランドマークの位置・姿勢
            t_robot2landmark = t_map2robot.inverse() * t_map2landmark
            robot2landmark = almath.position6DFromTransform(t_robot2landmark)
            #print(id, robot2landmark, robot2landmark.norm())
            
            local_angle_xy, local_angle_xz = self._get_angle_vector(robot2landmark)

            if (side_range[0] < local_angle_xy < side_range[1]) and (side_range[0] < local_angle_xz < side_range[1]):
                if robot2landmark.norm() < min_distance:
                    min_distance = robot2landmark.norm()
                    min_landmark_id = id
            #print(math.degrees(local_angle_xy))
        return min_landmark_id

    # 上の関数でどのランドマークも条件に当てはまらなかった場合に呼ばれる
    # ランドマーク全てに対して距離となす角度(xy, xz二通り)を計算し、もっとも移動量の少なくて済む
    # ランドマークを取得する
    # 条件順位：下記条件を満たすランドマークを取得する TODO: この優先順位だとnullが帰ってくるパターンがある
    # 1. xz面において任意の角度内に収まっている
    # 2. xy面においてベクトル同士のなす各がすべてのランドマークにたいして最小
    # 3. ランドマークとロボット間の距離が認識できる最大・最小距離内に収まっている
    def get_closest_landmark(self, robot_pose, landmark):
        distance_info = dict()
        side_range = [-math.radians(70), math.radians(70)]
        for id in landmark:
            # calculate angle robot to landmark
            # マップ原点座標から見たランドマークの位置・姿勢
            map2landmark = almath.Position6D(landmark[id])
            t_map2landmark = almath.transformFromPosition6D(map2landmark)
            # map原点座標から見たロボットの位置・姿勢
            map2robot = almath.position6DFromPose2D(robot_pose)
            t_map2robot = almath.transformFromPosition6D(map2robot)
            # ロボットから見たランドマークの位置・姿勢
            t_robot2landmark = t_map2robot.inverse() * t_map2landmark
            robot2landmark = almath.position6DFromTransform(t_robot2landmark)

            local_angle_xy, local_angle_xz = self._get_angle_vector(robot2landmark)

            # 1. xz面において任意の角度内に収まっている
            # TODO この時点でnullが帰ってくる可能性がある
            print(math.degrees(local_angle_xz))
            #if (side_range[0] < local_angle_xz < side_range[1]):
                # ベクトルのノルムを格納する
            distance_info[id] = local_angle_xy#robot2landmark.norm()

        # 2. xy面においてベクトル同士のなす角がすべてのランドマークに対して最小か
        #    1.でフィルタしたランドマークリストを元に角度が小さい順にソートする
        #min_k = min(d, key=d.get)
        #min_landmark_id = min(distance_info, key=distance_info.get)
        distance_info = sorted(distance_info.items(), key=lambda x:x[1])
        print(distance_info)
        return distance_info

        # 3. 一番角度が小さいランドマークを取得する。この時3.の認識できる距離の範囲内にランドマークが存在しているかをチェックする

    def _get_angle_vector(self, robot2landmark):
        # ロボットのローカル座標を示す一時的な位置ベクトル
        local_vector = almath.position6DFromPose2D(almath.Pose2D(1.0, 0, 0))
        # ロボットとランドマークのなす角度を計算する(x,y平面)
        a1 = local_vector.x*robot2landmark.x
        b1 = math.sqrt(local_vector.x*local_vector.x)*math.sqrt(robot2landmark.x*robot2landmark.x+robot2landmark.y*robot2landmark.y)
        local_angle_xy = math.acos(a1/b1)

        # ロボットとランドマークのなす角度を計算する(x,z平面)
        a2 = local_vector.x*robot2landmark.x
        b2 = math.sqrt(local_vector.x*local_vector.x)*math.sqrt(robot2landmark.x*robot2landmark.x+robot2landmark.z*robot2landmark.z)
        local_angle_xz = math.acos(a2/b2)

        return (local_angle_xy, local_angle_xz)

def load_landmark():
    txt = open("map.yaml", "r")
    marker_list = yaml.load(txt)
    return marker_list

if __name__ == "__main__":
    visualize = Visualize()
    estimate = EstimateLandmark()

    marker_list = list()
    marker_list = load_landmark()

    initial_robot_pose = almath.Pose2D(1,1,-almath.PI/2)
    while True:
        initial_robot_pose.theta = initial_robot_pose.theta + 0.05
        visualize.config_screen()

        closest_landmark_id = estimate.get_closest_landmark_non_move(initial_robot_pose, marker_list)
        if closest_landmark_id != -1:
            visualize.draw_circle(marker_list[closest_landmark_id][0], marker_list[closest_landmark_id][1])
        else:
            temp_landmark = estimate.get_closest_landmark(initial_robot_pose, marker_list)
            #closest_landmark_id = min(temp_landmark, key=temp_landmark.get)
            closest_landmark_id = temp_landmark[0][0]
            visualize.draw_circle(marker_list[closest_landmark_id][0], marker_list[closest_landmark_id][1])
        visualize.draw_landmark(marker_list)
        visualize.move_robot(initial_robot_pose.toVector())

        plt.pause(0.01)